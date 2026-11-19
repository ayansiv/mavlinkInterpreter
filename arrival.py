"""Optional arrival notifications and operator handoff for the SITL harness."""

import math
import threading
import time


CONTROL_TOOLS = {'set_mode', 'guided_goto', 'update_waypoint'}
MAX_AGE = 3


class ArrivalAssist:
    def __init__(self, vehicle, radius, distance, log, notify=print):
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Approach radius must be a positive, finite number of metres')
        self.vehicle, self.radius, self.distance = vehicle, radius, distance
        self.log, self.notify = log, notify
        self.lock = threading.RLock()
        self.local = threading.local()
        self.stop = threading.Event()
        self.thread = None
        self.generation = 0
        self.state = 'automation'
        self.reason = None
        self.destination = None
        self.notified = False
        self.expected_mode = vehicle.status().get('mode')
        self.pending_mode = None

    def event(self, name, **fields):
        self.log({'timestamp': time.time(), 'event': name, 'state': self.state, **fields})

    def snapshot(self):
        with self.lock:
            return {'state': self.state, 'reason': self.reason, 'approach_radius_m': self.radius,
                    'destination': dict(self.destination) if self.destination else None,
                    'approach_notified': self.notified}

    def token(self):
        with self.lock:
            return self.generation

    def current(self, generation):
        with self.lock:
            return not self.stop.is_set() and generation == self.generation

    @staticmethod
    def fresh_heartbeat(status):
        heartbeat = status.get('messages', {}).get('HEARTBEAT')
        return heartbeat is not None and 0 <= heartbeat['age_seconds'] <= MAX_AGE

    def suspend(self, reason):
        # Caller holds lock. Never silently recover when telemetry/mode recovers.
        self.generation += 1
        self.state, self.reason = 'blocked', reason
        self.destination, self.pending_mode = None, None
        self.event('control_suspended', reason=reason)
        self.notify('\n[assist] Model movement blocked: ' + reason + '. Use /manual to establish a handoff.')

    def observe(self, status):
        # Called under lock, including immediately before every control send.
        if self.state != 'automation':
            return
        if not self.fresh_heartbeat(status):
            self.suspend('Heartbeat missing or stale')
            return
        mode = status.get('mode')
        if self.pending_mode is not None and mode == self.pending_mode:
            self.expected_mode, self.pending_mode = mode, None
        elif mode != self.expected_mode:
            self.suspend('Unexpected flight mode: ' + str(mode))
            return
        position = status.get('messages', {}).get('GLOBAL_POSITION_INT')
        if not self.destination or self.notified or mode != 'GUIDED' or self.pending_mode is not None:
            return
        if not position or not 0 <= position['age_seconds'] <= MAX_AGE:
            return
        fields = position['fields']
        metres = 1000 * self.distance(fields['lat'] / 1e7, fields['lon'] / 1e7,
                                     self.destination['latitude'], self.destination['longitude'])
        if math.isfinite(metres) and metres <= self.radius:
            self.notified = True
            self.event('approach', distance_m=round(metres, 2), destination=dict(self.destination))
            self.notify('\n[assist] Within %.1f m horizontally of the Guided destination. '
                        'View the configured video in Mission Planner; /manual requests takeover. '
                        'This is an approach notification, not confirmation of arrival.' % metres)

    def check(self, generation):
        self.observe(self.vehicle.status())
        if self.stop.is_set() or generation != self.generation or self.state != 'automation':
            self.event('command_blocked', tool=getattr(self.local, 'tool', None),
                       reason=self.reason or 'Operator control or superseded model turn')
            raise RuntimeError('Model movement blocked by arrival assistance; use explicit operator commands')

    def call(self, name, arguments, generation, functions):
        self.local.generation, self.local.tool = generation, name
        try:
            with self.lock:
                if name in CONTROL_TOOLS:
                    self.check(generation)
                elif not self.current(generation):
                    raise RuntimeError('Model turn was superseded by an operator/state change')
            return functions[name](**arguments)
        finally:
            self.local.__dict__.clear()

    def send(self, function, *arguments, mode=None, destination=None):
        # The guard and actual MAVLink send share a lock with handoff requests.
        # Check again here: geocoding or a mission download may have taken time.
        with self.lock:
            generation = getattr(self.local, 'generation', None)
            if getattr(self.local, 'operator', False):
                if self.stop.is_set() or generation != self.generation or self.state != 'handoff':
                    raise RuntimeError('Operator handoff was superseded')
            else:
                self.check(generation)
            result = function(*arguments)
            if mode is not None and self.state == 'automation':
                self.pending_mode = mode
            if destination is not None:
                self.destination, self.notified = dict(destination), False
                self.event('destination_sent', destination=dict(destination))
            return result

    def mode_failed(self):
        with self.lock:
            if self.state == 'automation' and getattr(self.local, 'generation', None) == self.generation:
                self.suspend('Flight mode change was not confirmed')

    def manual(self):
        with self.lock:
            self.generation += 1
            generation = self.generation
            self.state, self.reason = 'handoff', None
            self.destination, self.pending_mode = None, None
            self.notified = False
            self.event('handoff_requested')
        self.notify('\n[assist] Model movement blocked. Requesting LOITER to leave Guided navigation.')
        self.local.generation, self.local.operator = generation, True
        try:
            if not self.fresh_heartbeat(self.vehicle.status()):
                raise RuntimeError('A fresh heartbeat is required for a handoff')
            self.vehicle.mode('LOITER')
            with self.lock:
                if not self.current(generation):
                    raise RuntimeError('Operator handoff was superseded')
                self.require_loiter()
                self.state = 'awaiting_operator'
                self.event('handoff_mode_confirmed', mode='LOITER')
            self.notify('[assist] LOITER confirmed. Verify the configured Mission Planner controller responds, '
                        'then enter /manual-ready. Mode confirmation does not verify the controller.')
        except Exception as exc:
            with self.lock:
                if self.current(generation):
                    self.state, self.reason = 'blocked', str(exc)
                    self.event('handoff_failed', reason=str(exc))
            raise
        finally:
            self.local.__dict__.clear()

    def require_loiter(self):
        status = self.vehicle.status()
        if not self.fresh_heartbeat(status) or status.get('mode') != 'LOITER':
            raise RuntimeError('A fresh LOITER heartbeat is required; verify the vehicle in Mission Planner')

    def manual_ready(self):
        with self.lock:
            if self.state != 'awaiting_operator':
                raise RuntimeError('Use /manual and wait for LOITER confirmation first')
            self.require_loiter()
            self.state = 'manual'
            self.event('operator_control_confirmed', confirmed_by='operator')
        self.notify('[assist] Operator confirmed controller operation. Model movement remains blocked.')

    def resume(self):
        with self.lock:
            if self.state != 'manual':
                raise RuntimeError('Complete /manual and /manual-ready before /resume')
            self.require_loiter()
            self.generation += 1
            self.state, self.reason = 'automation', None
            self.expected_mode, self.pending_mode = 'LOITER', None
            self.destination, self.notified = None, False
            self.event('automation_resumed')
        self.notify('[assist] New model movement commands are enabled. Vehicle remains in LOITER; '
                    'no previous destination or mission was resumed.')

    def poll(self):
        while not self.stop.wait(0.2):
            try:
                with self.lock:
                    self.observe(self.vehicle.status())
            except Exception as exc:
                with self.lock:
                    if self.state == 'automation':
                        self.suspend('Arrival monitor failed: ' + str(exc))

    def start(self):
        self.thread = threading.Thread(target=self.poll, daemon=True)
        self.thread.start()

    def close(self):
        with self.lock:
            self.stop.set()
            self.generation += 1
            self.state = 'closed'
            self.event('assistance_closed')
        if self.thread:
            self.thread.join(timeout=1)
