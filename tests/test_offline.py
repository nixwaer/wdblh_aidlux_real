#!/usr/bin/env python3
"""No services or devices: exercise config, wire formats and certificate creation."""
import importlib.util
import os
from pathlib import Path
import stat
import struct
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


daemon = load('wdblh_daemon', 'gamepad_bridge/wdblh_control_daemon.py')
server = load('wdblh_server', 'gamepad_bridge/server.py')
renderer = load('wdblh_renderer', 'scripts/render_wdblh_service.py')


class PortableConfiguration(unittest.TestCase):
    def test_environment_overrides_config_and_relocates_defaults(self):
        with tempfile.TemporaryDirectory(prefix='wdblh config ') as temp:
            config = Path(temp) / 'robot.env'
            config.write_text('WDBLH_MOTOR_BAUD=2000000\nWDBLH_BRIDGE_PORT=9443\n')
            env = {k: v for k, v in os.environ.items() if not k.startswith('WDBLH_')}
            env.update(WDBLH_ENV_FILE=str(config), WDBLH_PROJECT=temp,
                       WDBLH_MOTOR_BAUD='4000000')
            command = 'source "$1"; python3 -c \'import os,json;print(json.dumps({k:v for k,v in os.environ.items() if k.startswith("WDBLH_")}))\''
            result = subprocess.run(['bash', '-eu', '-c', command, 'test',
                                     str(ROOT/'scripts/wdblh_env.sh')], env=env,
                                    check=True, capture_output=True, text=True)
            import json
            values = json.loads(result.stdout)
            self.assertEqual(values['WDBLH_MOTOR_BAUD'], '4000000')
            self.assertEqual(values['WDBLH_BRIDGE_PORT'], '9443')
            self.assertEqual(values['WDBLH_CONTROL_LOG'], f'{temp}/logs/controller.log')
            self.assertEqual(values['WDBLH_POLICY_DIR'], f'{temp}/policy')

    def test_daemon_explicit_ports_win_over_detection(self):
        with mock.patch.multiple(daemon, IMU_PORT='/custom/imu', MOTOR_PORT='/custom/motor'), \
             mock.patch.object(daemon, '_resolve_port', side_effect=AssertionError('detection must not run')):
            self.assertEqual(daemon.control_args(), ['/custom/imu', '/custom/motor', '921600', '4000000'])

    def test_missing_explicit_motor_port_does_not_fall_back(self):
        with mock.patch.object(daemon, 'MOTOR_PORT', '/nonexistent-wdblh-motor'), \
             mock.patch.object(daemon, '_find_tty_by_usb_id', side_effect=AssertionError('unexpected fallback')):
            with self.assertRaises(FileNotFoundError):
                daemon.find_motor_port()

    def test_service_rendering_handles_spaces_and_specifiers(self):
        unit = renderer.render(Path('/tmp/robot % host'), 'robot')
        self.assertIn('ExecStart=/bin/bash "/tmp/robot %% host/scripts/wdblh_boot.sh"', unit)
        self.assertIn('WorkingDirectory=/tmp/robot %% host', unit)
        self.assertIn('Restart=no', unit)
        self.assertNotIn('@PROJECT@', unit)
        self.assertNotIn('/home/aidlux', unit)
        configured = renderer.render(ROOT, 'robot', Path('/tmp/robot config.env'))
        self.assertIn('Environment="WDBLH_ENV_FILE=/tmp/robot config.env"', configured)
        with self.assertRaises(ValueError):
            renderer.render(ROOT, 'bad\nUser=root')

    def test_shell_launcher_forwards_all_four_arguments_without_hardware(self):
        with tempfile.TemporaryDirectory() as temp:
            executable = Path(temp)/'fake-controller'
            executable.write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n')
            executable.chmod(0o755)
            env = {k: v for k, v in os.environ.items() if not k.startswith('WDBLH_')}
            env.update(WDBLH_CONTROL_BIN=str(executable), WDBLH_IMU_PORT='/fake/imu',
                       WDBLH_MOTOR_PORT='/fake/motor', WDBLH_IMU_BAUD='921600',
                       WDBLH_MOTOR_BAUD='4000000')
            result = subprocess.run(['bash', str(ROOT/'scripts/run_wdblh.sh')], env=env,
                                    check=True, text=True, capture_output=True)
            self.assertEqual(result.stdout.splitlines(), ['/fake/imu', '/fake/motor', '921600', '4000000'])


class Protocols(unittest.TestCase):
    def test_idle_and_damping_match_cpp_protocol(self):
        binary = os.environ.get('WDBLH_PROTOCOL_DUMP')
        if not binary:
            self.skipTest('set WDBLH_PROTOCOL_DUMP to offline test_protocol_dump executable')
        packets = subprocess.check_output([binary])
        self.assertEqual(len(packets), 584)
        self.assertEqual(daemon.idle_packet(), packets[:292])
        self.assertEqual(daemon.damping_packet(), packets[292:])

    def test_browser_gamepad_packet_without_network(self):
        fake_socket = mock.Mock()
        with mock.patch.object(server.socket, 'socket', return_value=fake_socket):
            relay = server.Relay('https://127.0.0.1:8443/', Path('/unused/ca.crt'))
        relay.send_to_robot({'gamepads': [{'axes': [2, -2, .25, float('nan')],
                                            'buttons': [{'pressed': True}]}]})
        data, address = fake_socket.sendto.call_args.args
        self.assertEqual(address, ('127.0.0.1', 9877))
        self.assertEqual(len(data), 28)
        self.assertEqual(struct.unpack('<4sI4fI', data), (b'GPB1', 1, 1, -1, .25, 0, 1))
        relay.send_to_robot(None)
        data = fake_socket.sendto.call_args.args[0]
        self.assertEqual(struct.unpack('<4sI4fI', data), (b'GPB1', 2, 0, 0, 0, 0, 0))

class LocalCertificates(unittest.TestCase):
    def test_new_keys_are_private_and_reused(self):
        from cryptography import x509
        with tempfile.TemporaryDirectory() as temp, mock.patch.object(server, 'CERT_DIR', Path(temp)/'certs'):
            cert, key, ca = server.ensure_certificates(['127.0.0.1', 'localhost'])
            first_key = key.read_bytes()
            for private in (key, key.parent/'ca-key.pem'):
                self.assertEqual(stat.S_IMODE(private.stat().st_mode), 0o600)
            parsed = x509.load_pem_x509_certificate(cert.read_bytes())
            self.assertIn('localhost', parsed.extensions.get_extension_for_class(x509.SubjectAlternativeName).value.get_values_for_type(x509.DNSName))
            server.ensure_certificates(['127.0.0.1', 'localhost'])
            self.assertEqual(key.read_bytes(), first_key)
            self.assertTrue(ca.is_file())


if __name__ == '__main__':
    unittest.main(verbosity=2)
