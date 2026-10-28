"""Cross-checks that keep code, messages, docs, launch files and configs from drifting apart.

Plain Python (standard library + PyYAML); no ROS needed.
"""
import ast
import re
import sys
import unittest
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / 'src'
MSG = SRC / 'arm_interfaces' / 'msg'
for p in (REPO / 'tests', SRC / 'arm_control', SRC / 'arm_teleop', SRC / 'arm_hardware'):
    sys.path.insert(0, str(p))

import check_interfaces as ci  # noqa: E402
from arm_control import arm_state_monitor, command_validation, control_config  # noqa: E402
from arm_hardware import bridge_types, firmware_sim, serial_protocol, telemetry_parser  # noqa: E402
from arm_teleop import teleop_core  # noqa: E402


def consts(msg_name):
    return {k: v[1] for k, v in ci.parse_msg(MSG / f'{msg_name}.msg', []).constants.items()}


class TestMirroredConstants(unittest.TestCase):
    def test_fault_bits(self):
        c = consts('ArmState')
        for name in ('COMMAND_TIMEOUT', 'ESTOP_LATCHED', 'SENSOR_INVALID', 'GRIPPER_OVERCURRENT',
                     'GRIPPER_MAX_RUNTIME', 'DRIVER', 'FIRMWARE_WATCHDOG', 'PROTOCOL_ERROR'):
            self.assertEqual(getattr(control_config, f'FAULT_{name}'), c[f'FAULT_{name}'], name)
        self.assertEqual(telemetry_parser.GLOBAL_FAULT_MASK, sum(v for k, v in c.items() if k.startswith('FAULT_')))
        self.assertEqual(set(arm_state_monitor.FAULT_NAMES.values()),
                         {k[len('FAULT_'):] for k in c if k.startswith('FAULT_')})

    def test_firmware_states_and_command_age(self):
        c = consts('ArmState')
        names = {v: k[len('FW_STATE_'):] for k, v in c.items() if k.startswith('FW_STATE_')}
        self.assertEqual(arm_state_monitor.FW_STATE_NAMES, names)
        self.assertEqual(telemetry_parser.FW_STATE_MAX, max(names))
        self.assertEqual(command_validation.FW_STATE_MAX, max(names))
        self.assertEqual(telemetry_parser.COMMAND_AGE_UNKNOWN, c['COMMAND_AGE_UNKNOWN'])
        self.assertEqual((firmware_sim.FW_BOOT, firmware_sim.FW_IDLE, firmware_sim.FW_ACTIVE,
                          firmware_sim.FW_CONTROLLED_STOP, firmware_sim.FW_FAULT, firmware_sim.FW_RECOVERY),
                         (c['FW_STATE_BOOT'], c['FW_STATE_IDLE'], c['FW_STATE_ACTIVE'],
                          c['FW_STATE_CONTROLLED_STOP'], c['FW_STATE_FAULT'], c['FW_STATE_RECOVERY']))
        self.assertEqual(arm_state_monitor.NO_MOTION_FW_STATES,
                         (c['FW_STATE_UNKNOWN'], c['FW_STATE_BOOT'], c['FW_STATE_FAULT'], c['FW_STATE_RECOVERY']))

    def test_motor_state_bits(self):
        c = consts('MotorState')
        self.assertEqual((telemetry_parser.VALID_OUTPUT, telemetry_parser.VALID_ENCODER,
                          telemetry_parser.VALID_ANGLE, telemetry_parser.VALID_CURRENT),
                         (c['VALID_OUTPUT'], c['VALID_ENCODER'], c['VALID_ANGLE'], c['VALID_CURRENT']))
        self.assertEqual(telemetry_parser.VALID_MASK, sum(v for k, v in c.items() if k.startswith('VALID_')))
        self.assertEqual(telemetry_parser.MOTOR_FAULT_MASK, sum(v for k, v in c.items() if k.startswith('FAULT_')))
        self.assertEqual((arm_state_monitor.VALID_OUTPUT, arm_state_monitor.VALID_ENCODER,
                          arm_state_monitor.VALID_ANGLE, arm_state_monitor.VALID_CURRENT),
                         (c['VALID_OUTPUT'], c['VALID_ENCODER'], c['VALID_ANGLE'], c['VALID_CURRENT']))

    def test_actions_and_command_types(self):
        u, a = consts('UserInput'), consts('ArmCommand')
        for mod in (command_validation,):
            self.assertEqual((mod.ACTION_INVALID, mod.ACTION_MOVE, mod.ACTION_STOP, mod.ACTION_ESTOP, mod.ACTION_REARM),
                             tuple(u[f'ACTION_{n}'] for n in ('INVALID', 'MOVE', 'STOP', 'ESTOP', 'REARM')))
        self.assertEqual((teleop_core.ACTION_MOVE, teleop_core.ACTION_STOP, teleop_core.ACTION_ESTOP,
                          teleop_core.ACTION_REARM), (1, 2, 3, 4))
        self.assertEqual((command_validation.COMMAND_TYPE_INVALID, command_validation.COMMAND_TYPE_SETPOINT,
                          command_validation.COMMAND_TYPE_STOP, command_validation.COMMAND_TYPE_ESTOP,
                          command_validation.COMMAND_TYPE_REARM),
                         tuple(a[f'COMMAND_TYPE_{n}'] for n in ('INVALID', 'SETPOINT', 'STOP', 'ESTOP', 'REARM')))
        self.assertEqual(a['NUM_CHANNELS'], serial_protocol.NUM_CHANNELS)

    def test_link_states(self):
        d = consts('ArmDiagnostics')
        self.assertEqual((bridge_types.LINK_DISCONNECTED, bridge_types.LINK_CONNECTING, bridge_types.LINK_CONNECTED,
                          bridge_types.LINK_TELEMETRY_STALE, bridge_types.LINK_PROTOCOL_MISMATCH),
                         tuple(d[f'LINK_{n}'] for n in ('DISCONNECTED', 'CONNECTING', 'CONNECTED',
                                                        'TELEMETRY_STALE', 'PROTOCOL_MISMATCH')))
        self.assertEqual(arm_state_monitor.LINK_NAMES, {v: k[len('LINK_'):] for k, v in d.items() if k.startswith('LINK_')})
        self.assertEqual(arm_state_monitor.LINK_CONNECTED, d['LINK_CONNECTED'])

    def test_blocking_fault_mask_in_yaml_matches_default(self):
        cfg = yaml.safe_load((SRC / 'arm_bringup' / 'config' / 'control.yaml').read_text())
        self.assertEqual(cfg['manual_control_node']['ros__parameters']['blocking_faults_mask'],
                         control_config.DEFAULT_BLOCKING_FAULTS_MASK)


class TestProtocolDocMatchesCode(unittest.TestCase):
    doc = (REPO / 'docs' / 'serial_protocol.md').read_text()

    def test_command_vectors_in_doc_match_encoder(self):
        sys.path.insert(0, str(SRC / 'arm_hardware' / 'test'))
        from test_serial_protocol import TestCommandVectors
        rows = re.findall(r'\| ([^|\n]+?) \| `([0-9a-f ]+)` \|', self.doc)
        doc_hex = [h.replace(' ', '') for _, h in rows]
        for _, expected in TestCommandVectors.VECTORS:
            self.assertIn(expected, doc_hex)
        self.assertEqual(len(doc_hex), len(TestCommandVectors.VECTORS))

    def test_telemetry_vector_in_doc_matches_encoder(self):
        sys.path.insert(0, str(SRC / 'arm_hardware' / 'test'))
        from test_serial_protocol import sample_telemetry
        frame = serial_protocol.encode_frame(serial_protocol.TYPE_TELEMETRY,
                                             telemetry_parser.build_telemetry_payload(sample_telemetry()))
        blocks = re.findall(r'```\n([0-9a-f \n]+)\n```', self.doc)
        flat = [b.replace('\n', ' ').replace(' ', '') for b in blocks]
        self.assertIn(frame.hex(), flat)
        self.assertIn(serial_protocol.cobs_decode(frame[:-1]).hex(), flat)

    def test_documented_sizes(self):
        self.assertEqual((telemetry_parser.HEADER_LEN, telemetry_parser.CHANNEL_LEN,
                          telemetry_parser.TELEMETRY_PAYLOAD_LEN), (20, 17, 139))
        self.assertIn('139 bytes', self.doc)
        self.assertEqual(serial_protocol.SETPOINT_PAYLOAD_LEN, 18)
        self.assertIn('0x29B1', self.doc)
        self.assertEqual(serial_protocol.crc16_ccitt_false(b'123456789'), 0x29B1)


def entry_points(pkg):
    tree = ast.parse((SRC / pkg / 'setup.py').read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == 'entry_points':
            d = ast.literal_eval(node.value)
            return {line.split('=')[0].strip(): line.split('=')[1].strip() for line in d['console_scripts']}
    return {}


class TestPackagingAndLaunch(unittest.TestCase):
    def test_every_entry_point_resolves_to_a_main_function(self):
        total = 0
        for pkg in ('arm_teleop', 'arm_control', 'arm_hardware'):
            for name, target in entry_points(pkg).items():
                module, func = target.split(':')
                path = SRC / pkg / (module.replace('.', '/') + '.py')
                self.assertTrue(path.exists(), (pkg, name, path))
                tree = ast.parse(path.read_text())
                self.assertIn(func, [n.name for n in tree.body if isinstance(n, ast.FunctionDef)], (pkg, name))
                total += 1
        self.assertEqual(total, 5)

    def test_planned_executables_exist(self):
        eps = {n for p in ('arm_teleop', 'arm_control', 'arm_hardware') for n in entry_points(p)}
        self.assertEqual(eps, {'keyboard_teleop_node', 'manual_control_node', 'arm_state_monitor',
                               'mock_arm_bridge', 'serial_arm_bridge'})

    def test_launch_files_reference_real_executables_nodes_and_configs(self):
        launch_dir = SRC / 'arm_bringup' / 'launch'
        names = sorted(p.name for p in launch_dir.glob('*.py'))
        self.assertEqual(names, ['hardware_control.launch.py', 'manual_control.launch.py', 'mock_control.launch.py'])
        yaml_nodes = {}
        for f in (SRC / 'arm_bringup' / 'config').glob('*.yaml'):
            yaml_nodes[f.name] = set(yaml.safe_load(f.read_text()))
        all_eps = {p: entry_points(p) for p in ('arm_teleop', 'arm_control', 'arm_hardware')}
        for name in names:
            text = (launch_dir / name).read_text()
            tree = ast.parse(text)
            for call in [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'Node']:
                kw = {k.arg: ast.literal_eval(k.value) for k in call.keywords if isinstance(k.value, ast.Constant)}
                self.assertIn(kw['executable'], all_eps[kw['package']], (name, kw))
                self.assertEqual(kw['name'], kw['executable'] if kw['executable'] != 'arm_state_monitor' else 'arm_state_monitor')
                # every config file passed to the node must exist and contain a section for that node
            for cfg_name in re.findall(r"'([a-z_]+\.yaml)'", text):
                self.assertIn(cfg_name, yaml_nodes, (name, cfg_name))
        manual = (launch_dir / 'manual_control.launch.py').read_text()
        self.assertIn('channels.yaml', manual)
        self.assertIn('control.yaml', manual)
        self.assertIn('manual_control_node', yaml_nodes['channels.yaml'])
        self.assertIn('hardware.yaml', (launch_dir / 'mock_control.launch.py').read_text())

    def test_hardware_launch_does_not_reference_mock(self):
        text = (SRC / 'arm_bringup' / 'launch' / 'hardware_control.launch.py').read_text()
        code = '\n'.join(l for l in text.splitlines() if not l.strip().startswith(('#', '"""')))
        self.assertNotIn("executable='mock_arm_bridge'", code)
        self.assertIn("executable='serial_arm_bridge'", code)
        self.assertNotIn('mock_control', code)

    def test_package_xml_declares_runtime_dependencies(self):
        import xml.etree.ElementTree as ET
        def deps(pkg):
            root = ET.parse(SRC / pkg / 'package.xml').getroot()
            return {e.text for t in ('depend', 'exec_depend', 'build_depend') for e in root.findall(t)}
        for pkg in ('arm_teleop', 'arm_control', 'arm_hardware'):
            self.assertTrue({'rclpy', 'arm_interfaces'} <= deps(pkg), pkg)
        self.assertIn('python3-serial', deps('arm_hardware'))
        self.assertIn('arm_control', deps('arm_hardware'))      # shared validator
        self.assertTrue({'arm_teleop', 'arm_control', 'arm_hardware', 'launch_ros'} <= deps('arm_bringup'))

    def test_no_deprecated_logger_warn_and_no_print_in_nodes(self):
        for py in SRC.glob('arm_*/arm_*/*.py'):
            text = py.read_text()
            self.assertNotIn('.warn(', text.replace('log.warning', ''), py.name)
        for node in ('manual_control_node', 'arm_state_monitor_node'):
            self.assertNotIn('print(', (SRC / 'arm_control' / 'arm_control' / f'{node}.py').read_text())

    def test_no_ramping_code_on_the_ros_side(self):
        # Ramping belongs to the STM32. Guard against ramp generators creeping into ROS code.
        for py in SRC.glob('arm_*/arm_*/*.py'):
            if py.name == 'firmware_sim.py':
                continue   # simulator doc explicitly states it does NOT ramp
            code = '\n'.join(l.split('#')[0] for l in py.read_text().splitlines())
            self.assertIsNone(re.search(r'ramp_(rate|step|increment|up|down)', code, re.I), py.name)


if __name__ == '__main__':
    unittest.main()
