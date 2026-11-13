import argparse
import json
import math
import threading
import time
from collections import deque

import requests
from pymavlink import mavutil


def valid_position(lat, lon, alt):
    vals = (lat, lon, alt)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in vals):
        raise ValueError("Coordinates and altitude must be finite numbers")
    if not -90 <= lat <= 90 or not -180 <= lon <= 180 or not -1000 <= alt <= 10000:
        raise ValueError("Position outside valid latitude, longitude or altitude range")


class Vehicle:
    def __init__(self, endpoint, timeout=5):
        self.link = mavutil.mavlink_connection(endpoint, source_system=255, source_component=190)
        self.timeout = timeout
        self.cv = threading.Condition()
        self.messages = {}
        self.events = deque(maxlen=1000)
        self.serial = 0
        self.target = None
        threading.Thread(target=self._read, daemon=True).start()
        deadline = time.monotonic() + timeout
        with self.cv:
            while self.target is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("No ArduPilot vehicle heartbeat. Check endpoint and MAVLink forwarding.")
                self.cv.wait(remaining)

    def _read(self):
        while True:
            msg = self.link.recv_match(blocking=True, timeout=1)
            if msg is None or msg.get_type() == 'BAD_DATA':
                continue
            with self.cv:
                if self.target is None and msg.get_type() == 'HEARTBEAT' and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA and msg.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
                    self.target = (msg.get_srcSystem(), msg.get_srcComponent())
                if self.target and (msg.get_srcSystem(), msg.get_srcComponent()) == self.target:
                    self.serial += 1
                    self.messages[msg.get_type()] = (time.monotonic(), msg)
                    self.events.append((self.serial, msg))
                    self.cv.notify_all()

    def marker(self):
        with self.cv:
            return self.serial

    def wait(self, types, after, predicate=lambda m: True):
        deadline = time.monotonic() + self.timeout
        with self.cv:
            while True:
                for serial, msg in self.events:
                    if serial > after and msg.get_type() in types and predicate(msg):
                        return serial, msg
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Timed out waiting for ' + '/'.join(types))
                self.cv.wait(remaining)

    def status(self):
        with self.cv:
            snapshot = dict(self.messages)
        result = {'target_system': self.target[0], 'target_component': self.target[1], 'messages': {}}
        now = time.monotonic()
        for name in ('HEARTBEAT', 'SYS_STATUS', 'BATTERY_STATUS', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'ATTITUDE', 'VFR_HUD', 'MISSION_CURRENT', 'STATUSTEXT'):
            if name in snapshot:
                stamp, msg = snapshot[name]
                data = msg.to_dict()
                data.pop('mavpackettype', None)
                result['messages'][name] = {'age_seconds': round(now - stamp, 2), 'fields': data}
        if 'HEARTBEAT' in snapshot:
            hb = snapshot['HEARTBEAT'][1]
            result['mode'] = mavutil.mode_string_v10(hb)
            result['armed'] = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return result

    def mode(self, mode):
        modes = self.link.mode_mapping() or mavutil.mode_mapping_acm
        mode = mode.upper()
        if mode not in modes:
            raise ValueError('Unknown Copter mode: ' + mode)
        mark = self.marker()
        self.link.mav.set_mode_send(self.target[0], mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, modes[mode])
        _, hb = self.wait(['HEARTBEAT'], mark, lambda m: m.custom_mode == modes[mode])
        return {'mode': mavutil.mode_string_v10(hb), 'confirmed_by': 'HEARTBEAT'}

    def mission(self):
        s, c = self.target
        for attempt in range(3):
            mark = self.marker()
            self.link.mav.mission_request_list_send(s, c)
            try:
                mark, count = self.wait(['MISSION_COUNT'], mark)
                break
            except TimeoutError:
                if attempt == 2: raise
        items = []
        for seq in range(count.count):
            for attempt in range(3):
                mark = self.marker()
                self.link.mav.mission_request_int_send(s, c, seq)
                try:
                    mark, item = self.wait(['MISSION_ITEM_INT', 'MISSION_ITEM'], mark, lambda m: m.seq == seq)
                    break
                except TimeoutError:
                    if attempt == 2: raise
            d = item.to_dict()
            if item.get_type() == 'MISSION_ITEM':
                d['x'], d['y'] = round(d['x'] * 1e7), round(d['y'] * 1e7)
            d.pop('mavpackettype', None)
            items.append(d)
        self.link.mav.mission_ack_send(s, c, mavutil.mavlink.MAV_MISSION_ACCEPTED)
        return items

    def update_waypoint(self, seq, latitude, longitude, altitude):
        valid_position(latitude, longitude, altitude)
        if isinstance(seq, bool) or not isinstance(seq, int): raise ValueError('seq must be an integer')
        items = self.mission()
        if seq <= 0 or seq >= len(items): raise ValueError('Sequence must identify an existing mission item after home')
        item = items[seq]
        if item['command'] != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            raise ValueError('Selected item is not a NAV_WAYPOINT')
        if item['frame'] not in (0, 3, 5, 6, 10, 11): raise ValueError('Waypoint is not in a supported global frame')
        item.update(x=round(latitude * 1e7), y=round(longitude * 1e7), z=altitude)
        s, c = self.target
        mark = self.marker()
        self.link.mav.mission_count_send(s, c, len(items))
        sent = set()
        for _ in range(len(items) * 4 + 4):
            try:
                mark, msg = self.wait(['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], mark)
            except TimeoutError:
                raise TimeoutError('Mission upload timed out; verify the vehicle mission before continuing')
            if msg.get_type() == 'MISSION_ACK':
                if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED or len(sent) != len(items):
                    raise RuntimeError(f'Mission upload failed or incomplete: ACK {msg.type}; verify vehicle mission')
                return {'updated_sequence': seq, 'result': 'accepted', 'altitude_frame': item['frame']}
            i = msg.seq
            if i >= len(items): raise RuntimeError('Vehicle requested invalid mission sequence')
            d = items[i]
            self.link.mav.mission_item_int_send(s, c, i, d['frame'], d['command'], d['current'], d['autocontinue'], d['param1'], d['param2'], d['param3'], d['param4'], d['x'], d['y'], d['z'])
            sent.add(i)
        raise RuntimeError('Too many mission requests; verify vehicle mission')

    def goto(self, latitude, longitude, altitude):
        valid_position(latitude, longitude, altitude)
        status = self.status()
        if status.get('mode') != 'GUIDED' or status['messages']['HEARTBEAT']['age_seconds'] > 3:
            raise ValueError('A fresh GUIDED mode heartbeat is required before goto')
        s, c = self.target
        self.link.mav.set_position_target_global_int_send(0, s, c, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0x0DF8, round(latitude * 1e7), round(longitude * 1e7), altitude, 0, 0, 0, 0, 0, 0, 0, 0)
        return {'result': 'sent (no MAVLink acknowledgement)', 'altitude_reference': 'metres above home'}


def schema(name, description, properties=None, required=None):
    return {'type': 'function', 'function': {'name': name, 'description': description, 'parameters': {'type': 'object', 'properties': properties or {}, 'required': required or []}}}

pos = {'latitude': {'type': 'number', 'description': 'Latitude in degrees'}, 'longitude': {'type': 'number', 'description': 'Longitude in degrees'}, 'altitude': {'type': 'number', 'description': 'Altitude in metres'}}
TOOLS = [schema('get_status', 'Get latest vehicle telemetry, including message age and raw MAVLink units.'), schema('get_mission', 'Download current mission items. Global x/y are latitude/longitude scaled by 1e7.'), schema('set_mode', 'Set an ArduCopter flight mode.', {'mode': {'type': 'string'}}, ['mode']), schema('update_waypoint', 'Change an existing NAV_WAYPOINT position while preserving its altitude frame and other mission items.', {'seq': {'type': 'integer'}, **pos}, ['seq', *pos]), schema('guided_goto', 'Send a Guided position target; altitude is metres above home. Requires GUIDED mode.', pos, list(pos))]
SYSTEM = 'You are a simulator flight-test assistant. Use tools for vehicle facts; never invent telemetry or command success. Raw MAVLink units and timestamps matter. Ask for missing coordinates, altitude or mode rather than guessing. For waypoint updates altitude uses the existing waypoint frame. Commands affect the simulator.'


def main():
    parser = argparse.ArgumentParser(description='Ollama / ArduCopter SITL MAVLink evaluation harness')
    parser.add_argument('--connect', default='udpin:127.0.0.1:14550', help='pymavlink endpoint')
    parser.add_argument('--model', required=True)
    parser.add_argument('--ollama', default='http://localhost:11434')
    parser.add_argument('--log', default='runs.jsonl')
    args = parser.parse_args()
    vehicle = Vehicle(args.connect)
    functions = {'get_status': vehicle.status, 'get_mission': vehicle.mission, 'set_mode': vehicle.mode, 'update_waypoint': vehicle.update_waypoint, 'guided_goto': vehicle.goto}
    print('Connected to system %s component %s. Enter a question, or /quit.' % vehicle.target)
    history = [{'role': 'system', 'content': SYSTEM}]
    while True:
        try: prompt = input('\n> ').strip()
        except (EOFError, KeyboardInterrupt): break
        if prompt in ('/quit', '/exit'): break
        if not prompt: continue
        history.append({'role': 'user', 'content': prompt})
        started = time.perf_counter()
        record = {'timestamp': time.time(), 'model': args.model, 'prompt': prompt, 'tools': []}
        try:
            for _ in range(8):
                call_start = time.perf_counter()
                response = requests.post(args.ollama.rstrip('/') + '/api/chat', json={'model': args.model, 'messages': history, 'tools': TOOLS, 'stream': False}, timeout=180)
                response.raise_for_status()
                data = response.json()
                msg = data['message']
                history.append(msg)
                record.setdefault('model_calls', []).append({'wall_seconds': round(time.perf_counter()-call_start, 3), 'total_duration_ns': data.get('total_duration'), 'eval_count': data.get('eval_count'), 'eval_duration_ns': data.get('eval_duration')})
                calls = msg.get('tool_calls') or []
                if not calls:
                    record['answer'] = msg.get('content', '')
                    print(record['answer'])
                    break
                for call in calls:
                    fn = call['function']
                    name, arguments = fn['name'], fn.get('arguments', {})
                    start = time.perf_counter()
                    try:
                        if isinstance(arguments, str): arguments = json.loads(arguments)
                        result = functions[name](**arguments)
                    except Exception as exc:
                        result = {'error': str(exc)}
                    record['tools'].append({'name': name, 'arguments': arguments, 'result': result, 'seconds': round(time.perf_counter()-start, 3)})
                    history.append({'role': 'tool', 'tool_name': name, 'content': json.dumps(result)})
            else:
                record['error'] = 'Tool-call limit reached'
                print(record['error'])
        except (requests.RequestException, ValueError, KeyError) as exc:
            record['error'] = str(exc)
            print('Error:', exc)
            history = [{'role': 'system', 'content': SYSTEM}]
        record['elapsed_seconds'] = round(time.perf_counter()-started, 3)
        with open(args.log, 'a', encoding='utf-8') as f: f.write(json.dumps(record) + '\n')
        print('[%.3fs; %d tool calls]' % (record['elapsed_seconds'], len(record['tools'])))


if __name__ == '__main__':
    main()
