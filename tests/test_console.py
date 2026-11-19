"""End-to-end console check with local protocol peers, not a flight simulator."""

import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymavlink import mavutil


class ConsoleTests(unittest.TestCase):
    def test_takeover_and_exit_while_model_request_is_pending(self):
        stop = threading.Event()
        delayed = threading.Event()
        model_entered = threading.Event()
        model_release = threading.Event()
        commands, output = [], []
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(0.1)

        def vehicle_peer():
            connection = None
            try:
                while not stop.is_set():
                    try:
                        connection, _ = listener.accept()
                        break
                    except socket.timeout:
                        pass
                if connection is None: return
                connection.settimeout(0.05)

                class Writer:
                    def write(self, data):
                        connection.sendall(data)

                mav = mavutil.mavlink.MAVLink(Writer(), srcSystem=1, srcComponent=1)
                mode = 4
                while not stop.is_set():
                    mav.heartbeat_send(2, 3, 1, mode, 3)
                    mav.global_position_int_send(1000, -350000000, 1490000000, 20000, 0, 0, 0, 0, 0)
                    try:
                        data = connection.recv(65536)
                        if not data: break
                        for message in mav.parse_buffer(data) or []:
                            commands.append(message.get_type())
                            if message.get_type() == 'SET_MODE': mode = message.custom_mode
                    except socket.timeout:
                        pass
            except OSError:
                pass  # Expected when the test tears down the local peer.
            finally:
                if connection: connection.close()

        class ModelPeer(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                if delayed.is_set():
                    model_entered.set()
                    model_release.wait(10)
                    message = {'role': 'assistant', 'tool_calls': [
                        {'function': {'name': 'set_mode', 'arguments': {'mode': 'GUIDED'}}}]}
                elif body['messages'][-1]['role'] == 'tool':
                    message = {'role': 'assistant', 'content': 'Test destination sent.'}
                else:
                    message = {'role': 'assistant', 'tool_calls': [
                        {'function': {'name': 'guided_goto', 'arguments': {
                            'latitude': -35, 'longitude': 149, 'altitude': 20}}}]}
                content = json.dumps({'message': message}).encode()
                try:
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                except OSError:
                    pass  # The harness may exit while this response is pending.

        model = ThreadingHTTPServer(('127.0.0.1', 0), ModelPeer)
        vehicle_thread = threading.Thread(target=vehicle_peer, daemon=True)
        model_thread = threading.Thread(target=model.serve_forever, daemon=True)
        vehicle_thread.start()
        model_thread.start()
        process = None
        try:
            with tempfile.TemporaryDirectory(prefix='mavlink-console-test-') as directory:
                log_path = Path(directory) / 'runs.jsonl'
                process = subprocess.Popen([
                    sys.executable, '-u', str(Path(__file__).resolve().parents[1] / 'harness.py'),
                    '--connect', 'tcp:127.0.0.1:%d' % listener.getsockname()[1],
                    '--model', 'local-test', '--ollama', 'http://127.0.0.1:%d' % model.server_port,
                    '--approach-radius', '30', '--log', str(log_path),
                ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                output_thread = threading.Thread(target=lambda: output.extend(iter(process.stdout.readline, '')), daemon=True)
                output_thread.start()

                def records():
                    if not log_path.exists(): return []
                    result = []
                    for line in log_path.read_text().splitlines():
                        try: result.append(json.loads(line))
                        except json.JSONDecodeError: pass  # A concurrent write may be incomplete.
                    return result

                def wait_for(predicate):
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        if predicate(): return
                        if process.poll() is not None: break
                        stop.wait(0.02)
                    self.fail('Console condition timed out:\n' + ''.join(output))

                def command(value):
                    process.stdin.write(value + '\n')
                    process.stdin.flush()

                def has_event(name):
                    return any(r.get('event') == name for r in records())

                wait_for(lambda: any('Arrival assistance enabled' in line for line in output))
                command('Send a test destination')
                wait_for(lambda: has_event('approach') and any(r.get('answer') for r in records()))
                self.assertEqual(commands.count('SET_POSITION_TARGET_GLOBAL_INT'), 1)

                delayed.set()
                command('A delayed request')
                self.assertTrue(model_entered.wait(5))
                command('/manual')
                wait_for(lambda: has_event('handoff_mode_confirmed'))
                self.assertFalse(model_release.is_set())
                self.assertEqual(commands.count('SET_MODE'), 1)
                command('/manual-ready')  # Synthetic operator acknowledgement; no physical controller.
                wait_for(lambda: has_event('operator_control_confirmed'))
                command('/resume')
                wait_for(lambda: has_event('automation_resumed'))
                model_release.set()
                wait_for(lambda: any(r.get('cancelled') for r in records()))
                self.assertEqual(commands.count('SET_MODE'), 1)  # No late GUIDED mode command.
                self.assertEqual(commands.count('SET_POSITION_TARGET_GLOBAL_INT'), 1)

                model_entered.clear()
                model_release.clear()
                command('Another delayed request')
                self.assertTrue(model_entered.wait(5))
                command('/quit')
                self.assertEqual(process.wait(timeout=3), 0)
                self.assertTrue(has_event('assistance_closed'))
                self.assertFalse(model_release.is_set())
                process.stdin.close()
                output_thread.join(1)
                process.stdout.close()
        finally:
            model_release.set()
            stop.set()
            if process and process.poll() is None:
                process.kill()
                process.wait(timeout=3)
            listener.close()
            vehicle_thread.join(1)
            model.shutdown()
            model.server_close()
            model_thread.join(1)


if __name__ == '__main__':
    unittest.main()
