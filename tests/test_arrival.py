import argparse
import copy
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from arrival import ArrivalAssist
from harness import Geocoder, SYSTEM, Vehicle, build_functions, distance_km, positive_metres, run_turn
from pymavlink import mavutil


class FakeVehicle(Vehicle):
    """Exercise the real command methods without a network or an aircraft."""
    def __init__(self):
        self.target = (1, 1)
        self.assistance = None
        self.send_lock = threading.Lock()
        self.link = SimpleNamespace(mav=Mock(), mode_mapping=lambda: {'GUIDED': 4, 'LOITER': 5, 'RTL': 6})
        self.telemetry = {'mode': 'GUIDED', 'messages': {
            'HEARTBEAT': {'age_seconds': 0, 'fields': {}},
            'GLOBAL_POSITION_INT': {'age_seconds': 0, 'fields': {'lat': -350000000, 'lon': 1490000000}}
        }}
        self.fail_mode = False

    def status(self):
        return copy.deepcopy(self.telemetry)

    def marker(self):
        return 0

    def wait(self, types, after, predicate=lambda m: True):
        if self.fail_mode:
            raise TimeoutError('No confirming heartbeat')
        mode = self.link.mav.set_mode_send.call_args.args[2]
        self.telemetry['mode'] = {4: 'GUIDED', 5: 'LOITER', 6: 'RTL'}[mode]
        hb = mavutil.mavlink.MAVLink_heartbeat_message(2, 3, 1, mode, 3, 3)
        return 1, hb


class ArrivalTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = FakeVehicle()
        self.records = []
        self.assist = ArrivalAssist(self.vehicle, 30, distance_km, self.records.append, Mock())
        self.vehicle.assistance = self.assist
        self.functions = build_functions(self.vehicle, Geocoder(None))

    def tearDown(self):
        self.assist.close()

    def goto(self, latitude=-35, longitude=149):
        return self.assist.call('guided_goto', {'latitude': latitude, 'longitude': longitude, 'altitude': 20},
                                self.assist.token(), self.functions)

    def observe(self):
        with self.assist.lock:
            self.assist.observe(self.vehicle.status())

    def events(self, name):
        return [r for r in self.records if r.get('event') == name]

    def handoff(self):
        self.assist.manual()
        self.assist.manual_ready()

    def test_radius_validation(self):
        for value in ('0', '-1', 'nan', 'inf', 'nope'):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                positive_metres(value)
        self.assertEqual(positive_metres('30.5'), 30.5)

    def test_approach_once_per_destination_and_reset(self):
        self.goto()
        self.observe()
        self.observe()
        self.assertEqual(len(self.events('approach')), 1)
        self.goto()
        self.observe()
        self.assertEqual(len(self.events('approach')), 2)

    def test_only_latest_destination_is_tracked(self):
        self.goto(-36, 149)
        self.goto(-37, 149)
        self.vehicle.telemetry['messages']['GLOBAL_POSITION_INT']['fields']['lat'] = -360000000
        self.observe()
        self.assertFalse(self.events('approach'))
        self.vehicle.telemetry['messages']['GLOBAL_POSITION_INT']['fields']['lat'] = -370000000
        self.observe()
        self.assertEqual(self.events('approach')[0]['destination']['latitude'], -37)

    def test_stale_or_missing_position_does_not_notify(self):
        self.goto()
        position = self.vehicle.telemetry['messages']['GLOBAL_POSITION_INT']
        position['age_seconds'] = 4
        self.observe()
        del self.vehicle.telemetry['messages']['GLOBAL_POSITION_INT']
        self.observe()
        self.assertFalse(self.events('approach'))
        position['age_seconds'] = 0
        self.vehicle.telemetry['messages']['GLOBAL_POSITION_INT'] = position
        self.observe()
        self.assertEqual(len(self.events('approach')), 1)

    def test_stale_heartbeat_blocks_until_explicit_handoff(self):
        self.goto()
        self.vehicle.telemetry['messages']['HEARTBEAT']['age_seconds'] = 4
        self.observe()
        self.assertEqual(self.assist.state, 'blocked')
        self.assertFalse(self.events('approach'))
        self.vehicle.telemetry['messages']['HEARTBEAT']['age_seconds'] = 0
        with self.assertRaises(RuntimeError): self.goto()
        self.assertIsNone(self.assist.snapshot()['destination'])

    def test_failed_send_does_not_register_destination(self):
        self.vehicle.link.mav.set_position_target_global_int_send.side_effect = OSError('link lost')
        with self.assertRaises(OSError): self.goto()
        self.assertIsNone(self.assist.snapshot()['destination'])

    def test_model_mode_transition_is_not_mistaken_for_external_takeover(self):
        self.assist.call('set_mode', {'mode': 'LOITER'}, self.assist.token(), self.functions)
        self.observe()
        self.assertEqual(self.assist.state, 'automation')
        self.assist.call('set_mode', {'mode': 'GUIDED'}, self.assist.token(), self.functions)
        self.observe()
        self.goto()

    def test_external_mode_change_blocks_before_next_send_even_without_poll(self):
        self.vehicle.telemetry['mode'] = 'RTL'
        with self.assertRaises(RuntimeError): self.goto()
        self.vehicle.link.mav.set_position_target_global_int_send.assert_not_called()
        self.assertEqual(self.assist.state, 'blocked')

    def test_unconfirmed_model_mode_change_blocks(self):
        self.vehicle.fail_mode = True
        with self.assertRaises(TimeoutError):
            self.assist.call('set_mode', {'mode': 'LOITER'}, self.assist.token(), self.functions)
        self.assertEqual(self.assist.state, 'blocked')

    def test_manual_handoff_requires_mode_and_operator_confirmation(self):
        self.assist.manual()
        self.assertEqual(self.assist.state, 'awaiting_operator')
        self.assertEqual(self.vehicle.link.mav.set_mode_send.call_args.args[2], 5)
        with self.assertRaises(RuntimeError): self.assist.resume()
        self.assist.manual_ready()
        self.assertEqual(self.assist.state, 'manual')
        self.assertEqual(self.events('operator_control_confirmed')[0]['confirmed_by'], 'operator')

    def test_failed_handoff_remains_blocked(self):
        self.vehicle.fail_mode = True
        with self.assertRaises(TimeoutError): self.assist.manual()
        self.assertEqual(self.assist.state, 'blocked')
        with self.assertRaises(RuntimeError): self.assist.manual_ready()
        with self.assertRaises(RuntimeError): self.assist.resume()

    def test_stale_handoff_sends_nothing(self):
        self.vehicle.telemetry['messages']['HEARTBEAT']['age_seconds'] = 4
        with self.assertRaises(RuntimeError): self.assist.manual()
        self.vehicle.link.mav.set_mode_send.assert_not_called()
        self.assertEqual(self.assist.state, 'blocked')

    def test_resume_does_not_replay_destination_or_change_mode(self):
        self.goto()
        self.handoff()
        self.vehicle.link.mav.reset_mock()
        self.assist.resume()
        self.assertEqual(self.vehicle.telemetry['mode'], 'LOITER')
        self.assertIsNone(self.assist.snapshot()['destination'])
        self.assertFalse(self.vehicle.link.mav.mock_calls)

    def test_resume_requires_current_loiter(self):
        self.handoff()
        self.vehicle.telemetry['mode'] = 'RTL'
        with self.assertRaises(RuntimeError): self.assist.resume()
        self.assertEqual(self.assist.state, 'manual')

    def test_all_model_mutations_blocked_but_status_remains_available(self):
        self.handoff()
        for name, arguments in (
            ('guided_goto', {'latitude': -35, 'longitude': 149, 'altitude': 20}),
            ('set_mode', {'mode': 'GUIDED'}),
            ('update_waypoint', {'seq': 1, 'latitude': -35, 'longitude': 149, 'altitude': 20}),
        ):
            with self.subTest(tool=name), self.assertRaises(RuntimeError):
                self.assist.call(name, arguments, self.assist.token(), self.functions)
        result = self.assist.call('get_status', {}, self.assist.token(), self.functions)
        self.assertEqual(result['arrival_assistance']['state'], 'manual')

    def test_handoff_during_geocoding_blocks_the_eventual_send_even_after_resume(self):
        entered, release = threading.Event(), threading.Event()
        errors = []

        def forward(location):
            entered.set()
            if not release.wait(3): raise TimeoutError('test geocoder did not release')
            return {'latitude': -35, 'longitude': 149, 'resolved_address': 'Test site'}

        functions = build_functions(self.vehicle, SimpleNamespace(forward=forward))
        generation = self.assist.token()

        def command():
            try:
                self.assist.call('guided_goto', {'location': 'Test site', 'altitude': 20}, generation, functions)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=command)
        worker.start()
        try:
            self.assertTrue(entered.wait(2))
            self.handoff()
            self.assist.resume()
            self.assist.call('set_mode', {'mode': 'GUIDED'}, self.assist.token(), self.functions)
        finally:
            release.set()
            worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.vehicle.link.mav.set_position_target_global_int_send.assert_not_called()

    def test_handoff_during_mission_upload_blocks_later_items(self):
        entered, release = threading.Event(), threading.Event()
        errors = []
        item = dict(frame=3, command=16, current=0, autocontinue=1,
                    param1=0, param2=0, param3=0, param4=0, x=0, y=0, z=20)
        self.vehicle.mission = lambda: [dict(item), dict(item)]
        heartbeat_wait = self.vehicle.wait

        def wait(types, after, predicate=lambda m: True):
            if types == ['HEARTBEAT']: return heartbeat_wait(types, after, predicate)
            entered.set()
            if not release.wait(3): raise TimeoutError('test upload did not release')
            return 1, SimpleNamespace(seq=0, get_type=lambda: 'MISSION_REQUEST_INT')

        self.vehicle.wait = wait
        generation = self.assist.token()

        def command():
            try:
                self.assist.call('update_waypoint', {'seq': 1, 'latitude': -35, 'longitude': 149, 'altitude': 20}, generation, self.functions)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=command)
        worker.start()
        try:
            self.assertTrue(entered.wait(2))
            self.handoff()
        finally:
            release.set()
            worker.join(3)
        self.assertEqual(len(errors), 1)
        self.assertIn('verify the mission', str(errors[0]))
        self.vehicle.link.mav.mission_count_send.assert_called_once()
        self.vehicle.link.mav.mission_item_int_send.assert_not_called()

    def test_late_model_response_is_discarded_and_history_reset(self):
        entered, release = threading.Event(), threading.Event()
        args = SimpleNamespace(model='test', ollama='http://unused')
        history = [{'role': 'system', 'content': SYSTEM}]
        logs = []
        generation = self.assist.token()

        def response(*args, **kwargs):
            entered.set()
            if not release.wait(3): raise TimeoutError('test model did not release')
            return Mock(json=lambda: {'message': {'role': 'assistant', 'tool_calls': [
                {'function': {'name': 'set_mode', 'arguments': {'mode': 'GUIDED'}}}]}})

        with patch('harness.requests.post', side_effect=response), patch('builtins.print'):
            worker = threading.Thread(target=run_turn, args=('Go', args, self.functions, history, logs.append, self.assist, generation))
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                self.handoff()
                self.assist.resume()
                self.vehicle.link.mav.reset_mock()
            finally:
                release.set()
                worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertFalse(self.vehicle.link.mav.mock_calls)
        self.assertIn('cancelled', logs[0])
        self.assertEqual(history, [{'role': 'system', 'content': SYSTEM}])

    def test_monitor_notifies_without_console_or_model_activity(self):
        notified = threading.Event()
        self.assist.notify = lambda message: notified.set() if 'horizontally' in message else None
        self.goto()
        self.assist.start()
        self.assertTrue(notified.wait(2))

    def test_shutdown_invalidates_pending_commands(self):
        generation = self.assist.token()
        self.assist.close()
        with self.assertRaises(RuntimeError):
            self.assist.call('set_mode', {'mode': 'GUIDED'}, generation, self.functions)
        self.vehicle.link.mav.set_mode_send.assert_not_called()

    def test_feature_disabled_preserves_direct_command_path(self):
        self.vehicle.assistance = None
        result = self.functions['guided_goto'](latitude=-35, longitude=149, altitude=20)
        self.assertEqual(result['result'], 'sent (no MAVLink acknowledgement)')
        self.vehicle.link.mav.set_position_target_global_int_send.assert_called_once()
        self.assertNotIn('arrival_assistance', self.functions['get_status']())

    def test_default_model_turn_still_executes_tools_and_logs_answer(self):
        self.vehicle.assistance = None
        responses = [
            Mock(json=lambda: {'message': {'role': 'assistant', 'tool_calls': [
                {'function': {'name': 'get_status', 'arguments': {}}}]}}),
            Mock(json=lambda: {'message': {'role': 'assistant', 'content': 'GUIDED'}}),
        ]
        logs = []
        with patch('harness.requests.post', side_effect=responses), patch('builtins.print'):
            run_turn('Status?', SimpleNamespace(model='test', ollama='http://unused'), self.functions,
                     [{'role': 'system', 'content': SYSTEM}], logs.append)
        self.assertEqual(logs[0]['answer'], 'GUIDED')
        self.assertEqual(logs[0]['tools'][0]['name'], 'get_status')


if __name__ == '__main__':
    unittest.main()
