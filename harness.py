import argparse
import json
import math
import os
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


GLOBAL_FRAMES = (0, 3, 5, 6, 10, 11)


def distance_km(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 6371.0 * 2 * math.asin(math.sqrt(a))


class Geocoder:
    """Google Maps Geocoding API: place name/address <-> latitude/longitude."""
    URL = 'https://maps.googleapis.com/maps/api/geocode/json'

    def __init__(self, key, timeout=10):
        self.key = key
        self.timeout = timeout
        self.reverse_cache = {}

    def _query(self, **params):
        if not self.key:
            raise RuntimeError('GOOGLE_MAPS_API environment variable is not set; location names cannot be resolved')
        try:
            r = requests.get(self.URL, params={**params, 'key': self.key}, timeout=self.timeout)
            r.raise_for_status()
            data = r.json()
        except (requests.RequestException, ValueError) as exc:
            # Don't pass the exception text on: it can contain the request URL, including the API key.
            code = getattr(getattr(exc, 'response', None), 'status_code', None)
            raise RuntimeError(f'Google geocoding request failed ({type(exc).__name__}{", HTTP %s" % code if code else ""})') from None
        status = data.get('status')
        if status == 'ZERO_RESULTS':
            return []
        if status != 'OK':
            raise RuntimeError(f"Google geocoding failed: {status} {data.get('error_message', '')}".strip())
        return data['results']

    def forward(self, location):
        if not isinstance(location, str) or not location.strip():
            raise ValueError('location must be a non-empty place name or address')
        results = self._query(address=location.strip())
        if not results:
            raise ValueError(f'No location found matching {location!r}')
        if len(results) > 1:
            options = [r.get('formatted_address') for r in results[:5]]
            raise ValueError(f'{location!r} is ambiguous; ask the user which they mean (or for a more specific address). Candidates: {options}')
        best = results[0]
        loc = best['geometry']['location']
        place = {'query': location, 'resolved_address': best.get('formatted_address'), 'latitude': loc['lat'], 'longitude': loc['lng']}
        if best.get('partial_match'):
            place['warning'] = 'Partial match only; confirm the resolved address with the user'
        return place

    def reverse(self, latitude, longitude):
        key = (round(latitude, 5), round(longitude, 5))
        if key not in self.reverse_cache:
            results = self._query(latlng=f'{latitude},{longitude}')
            self.reverse_cache[key] = results[0].get('formatted_address') if results else None
        return self.reverse_cache[key]


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
        if item['frame'] not in GLOBAL_FRAMES: raise ValueError('Waypoint is not in a supported global frame')
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

pos = {'latitude': {'type': 'number', 'description': 'Latitude in degrees (omit if location is given)'}, 'longitude': {'type': 'number', 'description': 'Longitude in degrees (omit if location is given)'}, 'location': {'type': 'string', 'description': "Place name or street address, exactly as the user said it, used instead of latitude/longitude"}, 'altitude': {'type': 'number', 'description': 'Altitude in metres'}}
TOOLS = [schema('get_status', 'Get latest vehicle telemetry, including message age and raw MAVLink units.'), schema('get_mission', 'Download current mission items. Global x/y are latitude/longitude scaled by 1e7; positional items also include decimal latitude/longitude and, when available, the nearest street address.'), schema('set_mode', 'Set an ArduCopter flight mode.', {'mode': {'type': 'string'}}, ['mode']), schema('update_waypoint', 'Change an existing NAV_WAYPOINT position while preserving its altitude frame and other mission items. Give either latitude+longitude or location.', {'seq': {'type': 'integer'}, **pos}, ['seq', 'altitude']), schema('guided_goto', 'Send a Guided position target; altitude is metres above home. Requires GUIDED mode. Give either latitude+longitude or location.', pos, ['altitude'])]
SYSTEM = ('You are a simulator flight-test assistant. Use tools for vehicle facts; never invent telemetry or command success. Raw MAVLink units and timestamps matter. '
          'Ask for missing coordinates, altitude or mode rather than guessing. For waypoint updates altitude uses the existing waypoint frame. Commands affect the simulator. '
          'When the user names a place or address instead of coordinates, pass their words in the location argument; never make up coordinates for a place yourself. '
          'After a location-based command, tell the user the resolved_address and distance_from_vehicle_km the tool returned. '
          'When describing waypoints, include their address from get_mission when one is present.')


def build_functions(vehicle, geocoder):
    def resolve(latitude, longitude, location):
        if location:
            place = geocoder.forward(location)
            latitude, longitude = place['latitude'], place['longitude']
        elif latitude is None or longitude is None:
            raise ValueError('Provide either latitude and longitude, or a location (place name or address)')
        else:
            place = None
        info = {'latitude': latitude, 'longitude': longitude}
        if place:
            info['resolved_address'] = place['resolved_address']
            if 'warning' in place: info['warning'] = place['warning']
        gpi = vehicle.status()['messages'].get('GLOBAL_POSITION_INT')
        if gpi:
            info['distance_from_vehicle_km'] = round(distance_km(gpi['fields']['lat'] / 1e7, gpi['fields']['lon'] / 1e7, latitude, longitude), 3)
        return latitude, longitude, info

    def get_mission():
        items = vehicle.mission()
        for item in items:
            if item['frame'] in GLOBAL_FRAMES and (item['x'] or item['y']):
                item['latitude'], item['longitude'] = item['x'] / 1e7, item['y'] / 1e7
                if geocoder.key:
                    try:
                        item['address'] = geocoder.reverse(item['latitude'], item['longitude'])
                    except Exception as exc:
                        item['address_error'] = str(exc)
        return items

    def update_waypoint(seq, altitude, latitude=None, longitude=None, location=None):
        latitude, longitude, info = resolve(latitude, longitude, location)
        return {**vehicle.update_waypoint(seq, latitude, longitude, altitude), **info}

    def guided_goto(altitude, latitude=None, longitude=None, location=None):
        latitude, longitude, info = resolve(latitude, longitude, location)
        return {**vehicle.goto(latitude, longitude, altitude), **info}

    return {'get_status': vehicle.status, 'get_mission': get_mission, 'set_mode': vehicle.mode, 'update_waypoint': update_waypoint, 'guided_goto': guided_goto}


def main():
    parser = argparse.ArgumentParser(description='Ollama / ArduCopter SITL MAVLink evaluation harness')
    parser.add_argument('--connect', default='udpin:127.0.0.1:14550', help='pymavlink endpoint')
    parser.add_argument('--model', required=True)
    parser.add_argument('--ollama', default='http://localhost:11434')
    parser.add_argument('--log', default='runs.jsonl')
    args = parser.parse_args()
    vehicle = Vehicle(args.connect)
    geocoder = Geocoder(os.environ.get('GOOGLE_MAPS_API'))
    functions = build_functions(vehicle, geocoder)
    print('Connected to system %s component %s. Enter a question, or /quit.' % vehicle.target)
    if not geocoder.key:
        print('Note: GOOGLE_MAPS_API is not set; place names and waypoint addresses are disabled.')
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
