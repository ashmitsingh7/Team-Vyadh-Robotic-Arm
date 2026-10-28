"""Node wiring and end-to-end pipeline tests using a FAKE rclpy and FAKE message classes.

*** These are NOT ROS tests. *** They run the real node classes (teleop, control, state
monitor, mock bridge, serial bridge) against an in-memory stand-in for rclpy, strict message
classes generated from the real .msg files, the real YAML configs, and a simulated STM32.
They catch logic, wiring, typing and configuration mistakes without ROS installed. They do
NOT prove `colcon build`, DDS behaviour, launch files, or anything about real hardware; run
`colcon build && colcon test` on the Ubuntu 24.04 + ROS 2 Jazzy machine for that.
"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / 'src'
CONFIG = SRC / 'arm_bringup' / 'config'
for p in (REPO / 'tests' / 'support', REPO / 'tests', SRC / 'arm_control', SRC / 'arm_teleop',
          SRC / 'arm_hardware'):
    sys.path.insert(0, str(p))

import fake_msgs  # noqa: E402
import fake_ros  # noqa: E402

fake_msgs.install()
fake_ros.install()

from arm_interfaces.msg import ArmCommand, ArmDiagnostics, ArmState, MotorState, UserInput  # noqa: E402

from arm_control.arm_state_monitor_node import ArmStateMonitorNode  # noqa: E402
from arm_control.command_validation import validate_arm_command  # noqa: E402
from arm_control.manual_control_node import ManualControlNode  # noqa: E402
from arm_hardware.fake_stm32 import FakeStm32  # noqa: E402
from arm_hardware.mock_arm_bridge import MockArmBridge  # noqa: E402
from arm_hardware.serial_arm_bridge import SerialArmBridge  # noqa: E402
from arm_teleop.keyboard_teleop_node import KeyboardTeleopNode  # noqa: E402

LINK_CONNECTED, LINK_DISCONNECTED, LINK_STALE = 2, 0, 3
FW_IDLE, FW_ACTIVE, FW_FAULT = 2, 3, 5
KEYS = {1: ('q', 'a'), 2: ('w', 's'), 3: ('e', 'd'), 4: ('r', 'f'), 5: ('t', 'g'), 6: ('y', 'h'), 7: ('u', 'j')}


class FakeKeys:
    def __init__(self):
        self.queue = []

    def read_keys(self):
        out, self.queue = self.queue, []
        return out


class BusSerial:
    """Serial port stand-in wired to a FakeStm32 that emits telemetry at 20 Hz of bus time."""

    def __init__(self, bus, fake, period=0.05):
        self.bus, self.fake, self.period = bus, fake, period
        self.next_tx = bus.now + period
        self.fail = False
        self.closed = False
        self.written = []

    def read(self, n):
        if self.fail:
            raise OSError('device disconnected')
        out = b''
        while self.bus.now + 1e-9 >= self.next_tx:
            self.next_tx += self.period
            out += self.fake.telemetry_frame(self.bus.now)
        return out[:n] if len(out) <= n else out

    def write(self, data):
        if self.fail:
            raise OSError('device disconnected')
        self.written.append(bytes(data))
        self.fake.receive(bytes(data), self.bus.now)

    def close(self):
        self.closed = True


class Pipeline:
    """teleop + control + monitor + (mock | serial) bridge on one fake bus with the real YAML."""

    def __init__(self, bridge='mock', overrides=None):
        self.bus = fake_ros.new_bus()
        for name in ('channels', 'control', 'teleop', 'hardware'):
            self.bus.load_yaml(CONFIG / f'{name}.yaml')
        for node, params in (overrides or {}).items():
            self.bus.overrides.setdefault(node, {}).update(params)
        self.keys = FakeKeys()
        self.control = ManualControlNode()
        self.monitor = ArmStateMonitorNode()
        self.teleop = KeyboardTeleopNode(self.keys)
        self.fake = None
        self.port = None
        self.opener_fails = False
        if bridge == 'mock':
            self.bridge = MockArmBridge()
        else:
            self.fake = FakeStm32()

            def opener():
                if self.opener_fails:
                    raise OSError('no such device /dev/ttyACM0')
                self.port = BusSerial(self.bus, self.fake)
                return self.port
            self.bridge = SerialArmBridge(opener=opener)
            # The real firmware keeps running (and its watchdog keeps timing) whether or not the
            # cable is connected, so the simulated STM32 gets its own clock independent of reads.
            clock = fake_ros.Node('fake_stm32_clock')
            clock.create_timer(0.01, lambda: self.fake.model.step(self.bus.now))

    # ---- helpers ----
    def advance(self, s):
        self.bus.advance(s)

    def hold(self, key, seconds, repeat=0.033):
        """Emulate a held terminal key: auto-repeat characters at ~30 Hz."""
        end = self.bus.now + seconds
        while self.bus.now < end - 1e-9:
            self.keys.queue.append(key)
            self.bus.advance(repeat)

    def tap(self, key, settle=0.3):
        self.keys.queue.append(key)
        self.bus.advance(settle)

    @property
    def state(self):
        return self.bus.messages('/arm/state')[-1]

    @property
    def diag(self):
        return self.bus.messages('/arm/diagnostics')[-1]

    def outputs(self):
        return [round(m.applied_output, 6) for m in sorted(self.state.motors, key=lambda m: m.channel)]

    def last_command(self):
        return self.bus.messages('/arm/command')[-1]

    def warm_up(self, s=0.6):
        self.advance(s)


class TestConfigMatchesNodes(unittest.TestCase):
    def test_every_yaml_parameter_is_declared_and_correctly_typed(self):
        p = Pipeline('mock')
        for node in (p.control, p.monitor, p.teleop, p.bridge):
            self.assertEqual(node.unused_overrides(), [], node.get_name())
        # serial bridge section of hardware.yaml
        p2 = Pipeline('serial')
        self.assertEqual(p2.bridge.unused_overrides(), [])

    def test_yaml_doubles_without_decimal_point_would_be_caught(self):
        with self.assertRaises(TypeError):
            Pipeline('mock', {'manual_control_node': {'channels.m1.max_demand': 1}})

    def test_channels_yaml_values_reach_the_node(self):
        p = Pipeline('mock')
        self.assertEqual(p.control.get_parameter('channels.m7.max_demand').value, 0.2)

    def test_invalid_configuration_is_refused(self):
        with self.assertRaises(ValueError):
            Pipeline('mock', {'manual_control_node': {'channels.m3.polarity': 0}})
        with self.assertRaises(ValueError):
            Pipeline('mock', {'keyboard_teleop_node': {'publish_rate_hz': 1.0, 'hold_window_s': 0.5}})


class TestMockPipeline(unittest.TestCase):
    def test_mock_mode_is_loud_and_all_data_simulated(self):
        p = Pipeline('mock')
        p.warm_up()
        self.assertTrue(any('MOCK MODE' in t for t in p.bus.log_text('warning')))
        self.assertTrue(all(m.simulated for m in p.bus.messages('/arm/state')))
        self.assertTrue(all(m.simulated for m in p.bus.messages('/arm/diagnostics')))
        self.assertEqual(p.diag.link_state, LINK_CONNECTED)

    def test_heartbeat_commands_are_valid_full_snapshots_at_20hz(self):
        p = Pipeline('mock')
        p.warm_up(1.0)
        cmds = p.bus.messages('/arm/command')
        self.assertGreaterEqual(len(cmds), 19)
        for c in cmds:
            self.assertEqual(validate_arm_command(c), [])
            self.assertEqual(len(c.motors), 7)
        self.assertEqual([c.sequence for c in cmds], list(range(len(cmds))))

    def test_each_of_seven_channels_both_directions_end_to_end(self):
        for ch in range(1, 8):
            for sign, key in zip((+1, -1), KEYS[ch]):
                p = Pipeline('mock')
                p.warm_up()
                p.hold(key, 0.4)
                out = p.outputs()
                self.assertAlmostEqual(out[ch - 1], sign * 0.1, places=6, msg=(ch, key))
                self.assertEqual(sum(1 for v in out if v), 1, (ch, key, out))
                self.assertEqual(p.state.firmware_state, FW_ACTIVE)

    def test_stop_latency_matches_hold_window(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('q', 0.5)
        self.assertNotEqual(p.outputs()[0], 0.0)
        t_release = p.bus.now
        while p.outputs()[0] != 0.0 and p.bus.now - t_release < 2.0:
            p.advance(0.005)
        latency = p.bus.now - t_release
        self.assertGreater(latency, 0.5)           # it really waits for the hold window
        self.assertLess(latency, 0.85)             # ... plus at most a few publish periods

    def test_space_stops_immediately(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('w', 0.4)
        p.tap(' ', 0.2)
        self.assertEqual(p.outputs(), [0.0] * 7)
        self.assertIn(2, [c.command_type for c in p.bus.messages('/arm/command')])   # STOP was sent

    def test_simultaneous_keys_including_independent_wrist_motors(self):
        p = Pipeline('mock')
        p.warm_up()
        for _ in range(12):
            p.keys.queue += ['t', 'h']          # M5 positive, M6 negative at the same time
            p.advance(0.033)
        out = p.outputs()
        self.assertEqual((out[4], out[5]), (0.1, -0.1))
        self.assertEqual(sum(1 for v in out if v), 2)

    def test_polarity_applies_per_channel(self):
        p = Pipeline('mock', {'manual_control_node': {'channels.m2.polarity': -1}})
        p.warm_up()
        p.hold('w', 0.4)                         # operator "+" on M2 with inverted polarity
        self.assertEqual(p.outputs()[1], -0.1)

    def test_level_keys_scale_demand(self):
        p = Pipeline('mock')
        p.warm_up()
        p.tap('+', 0.05)
        p.tap('+', 0.05)                         # 0.5 -> 0.7
        p.hold('q', 0.4)
        self.assertAlmostEqual(p.outputs()[0], 0.14, places=6)

    def test_estop_is_distinct_latches_and_rearm_never_starts_motion(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('q', 0.4)
        self.assertNotEqual(p.outputs()[0], 0.0)
        p.tap('x', 0.3)
        self.assertEqual(p.outputs(), [0.0] * 7)
        self.assertEqual(p.state.firmware_state, FW_FAULT)
        self.assertTrue(p.state.active_faults & 2)
        self.assertIn(3, [c.command_type for c in p.bus.messages('/arm/command')])
        p.hold('q', 0.5)                          # latched: key does nothing
        self.assertEqual(p.outputs(), [0.0] * 7)
        self.assertIn('ESTOP latched', ' '.join(p.bus.log_text('warning')))
        p.advance(1.0)
        p.tap('Z', 0.6)                           # re-arm
        self.assertEqual(p.outputs(), [0.0] * 7)  # REARM alone never moves anything
        self.assertFalse(p.state.active_faults & 2)
        self.assertIn(4, [c.command_type for c in p.bus.messages('/arm/command')])
        p.hold('q', 0.4)
        self.assertEqual(p.outputs()[0], 0.1)     # a NEW press moves again

    def test_estop_key_pressed_three_copies_published(self):
        p = Pipeline('mock')
        p.warm_up(0.2)
        p.tap('x', 0.1)
        estops = [m for m in p.bus.messages('/arm/user_input') if m.action == 3]
        self.assertEqual(len(estops), 3)
        self.assertEqual(len({m.sequence for m in estops}), 3)

    def test_key_held_through_estop_does_not_resume_after_rearm(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('q', 0.3)
        p.keys.queue.append('x')
        p.hold('q', 0.5)                           # physically still holding q
        p.keys.queue.append('Z')
        p.hold('q', 1.5)                           # still holding through REARM
        self.assertEqual(p.outputs(), [0.0] * 7)
        p.advance(1.0)                             # let go
        p.hold('q', 0.4)                           # press again
        self.assertEqual(p.outputs()[0], 0.1)

    def test_comm_loss_marks_stale_blocks_motion_and_never_auto_resumes(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('q', 0.4)
        self.assertEqual(p.outputs()[0], 0.1)
        p.bridge.set_param('simulate_comm_loss', True)
        n_states = len(p.bus.messages('/arm/state'))
        p.hold('q', 1.2)                           # operator keeps holding the key
        self.assertLessEqual(len(p.bus.messages('/arm/state')) - n_states, 1)   # telemetry stopped
        self.assertEqual(p.diag.link_state, LINK_STALE)
        self.assertTrue(any('STALE' in t or 'stale' in t for t in p.bus.log_text('warning')))
        self.assertTrue(all(m.demand == 0.0 for m in p.last_command().motors))   # control blocked itself
        p.bridge.set_param('simulate_comm_loss', False)
        p.hold('q', 2.0)                           # link back, key STILL held
        self.assertEqual(p.outputs(), [0.0] * 7)
        self.assertEqual(p.diag.link_state, LINK_CONNECTED)
        self.assertEqual(p.bus.messages('/arm/state')[-1].motors[0].fault_flags, 0)
        p.advance(1.0)                             # release
        p.hold('q', 0.4)                           # new press
        self.assertEqual(p.outputs()[0], 0.1)

    def test_firmware_timeout_when_commands_stop(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('q', 0.4)
        p.control.destroyed = True                 # control node dies: no more heartbeats
        p.teleop.destroyed = True
        p.advance(1.0)
        self.assertEqual(p.outputs(), [0.0] * 7)   # simulated firmware watchdog stopped it
        self.assertTrue(p.state.active_faults & 1)

    def test_invalid_sensor_is_visible_not_zero(self):
        p = Pipeline('mock')
        p.bridge.set_param('simulate_invalid_sensor_channel', 2)
        p.warm_up(1.6)                             # long enough for one monitor report
        m2 = sorted(p.state.motors, key=lambda m: m.channel)[1]
        self.assertEqual(m2.valid_flags & 4, 0)
        self.assertNotEqual(m2.joint_angle_rad, m2.joint_angle_rad)     # NaN
        self.assertTrue(p.state.active_faults & 4)
        self.assertIn('SENSOR_INVALID', ' '.join(p.bus.log_text()))

    def test_state_monitor_reports_simulated_and_stale(self):
        p = Pipeline('mock')
        p.advance(2.5)
        self.assertTrue(any('SIMULATED DATA' in t for t in p.bus.log_text('info')))
        p.bridge.set_param('simulate_comm_loss', True)
        p.advance(2.5)
        self.assertTrue(any('STALE' in t and 'BLOCKED' in t for t in p.bus.log_text('warning')))

    def test_malformed_state_is_ignored_not_trusted(self):
        p = Pipeline('mock')
        p.warm_up()
        bad = ArmState()
        bad.firmware_state = 3
        bad.motors = [MotorState(channel=1)]       # only one channel
        p.bus.enqueue('/arm/state', bad)
        p.bus.drain()
        self.assertTrue(any('malformed /arm/state' in t for t in p.bus.log_text('warning')))

    def test_garbage_user_input_never_moves_anything(self):
        p = Pipeline('mock')
        p.warm_up()
        for kw in (dict(action=0), dict(action=1, channel=9, direction=1, level=0.5),
                   dict(action=1, channel=1, direction=1, level=float('nan')),
                   dict(action=1, channel=1, direction=1, level=1.5), dict(action=7)):
            p.bus.enqueue('/arm/user_input', UserInput(**kw))
        p.advance(0.3)
        self.assertEqual(p.outputs(), [0.0] * 7)

    def test_teleop_publishes_stop_on_shutdown(self):
        p = Pipeline('mock')
        p.warm_up(0.2)
        p.keys.queue.append('q')
        p.advance(0.1)
        p.teleop.send_shutdown_stop()
        p.advance(0.1)
        self.assertEqual(p.bus.messages('/arm/user_input')[-1].action, 2)

    def test_restart_of_bridge_starts_at_rest(self):
        p = Pipeline('mock')
        p.warm_up()
        p.hold('q', 0.4)
        p.bridge.destroyed = True
        p.bridge = MockArmBridge()                 # restart
        p.advance(0.5)
        self.assertEqual(p.outputs(), [0.0] * 7)


class TestSerialPipelineWithSimulatedStm32(unittest.TestCase):
    """The REAL serial_arm_bridge node + protocol, talking to a simulated STM32 (not hardware)."""

    def test_connects_and_data_is_not_marked_simulated(self):
        p = Pipeline('serial')
        p.warm_up()
        self.assertEqual(p.diag.link_state, LINK_CONNECTED)
        self.assertFalse(p.diag.simulated)
        self.assertFalse(p.state.simulated)
        self.assertEqual(p.diag.local_protocol_version, 1)
        self.assertTrue(p.diag.remote_version_valid)

    def test_key_reaches_firmware_sim_and_acceptance_is_confirmed_by_telemetry(self):
        p = Pipeline('serial')
        p.warm_up()
        p.hold('q', 0.4)
        self.assertEqual(p.fake.model.applied[0], 0.1)
        self.assertEqual(p.outputs()[0], 0.1)
        cmd_seq = p.last_command().sequence
        self.assertTrue(p.state.accepted_sequence_valid)
        self.assertLessEqual(p.state.last_accepted_command_sequence, cmd_seq)
        self.assertGreater(p.state.last_accepted_command_sequence, 0)
        self.assertGreater(p.diag.frames_transmitted, 5)
        self.assertEqual(p.diag.frames_crc_error, 0)

    def test_all_seven_channels_through_the_real_protocol(self):
        for ch in range(1, 8):
            p = Pipeline('serial')
            p.warm_up()
            p.hold(KEYS[ch][1], 0.4)
            out = p.outputs()
            self.assertEqual(out[ch - 1], -0.1, ch)
            self.assertEqual(sum(1 for v in out if v), 1)

    def test_estop_over_serial_latches_in_firmware_sim(self):
        p = Pipeline('serial')
        p.warm_up()
        p.hold('q', 0.3)
        p.tap('x', 0.3)
        self.assertTrue(p.fake.model.estop_latched)
        self.assertEqual(p.outputs(), [0.0] * 7)
        self.assertEqual(p.diag.latched_faults & 2, 2)

    def test_cable_pull_is_visible_and_reconnect_does_not_resume_motion(self):
        p = Pipeline('serial')
        p.warm_up()
        p.hold('q', 0.4)
        self.assertEqual(p.outputs()[0], 0.1)
        p.port.fail = True
        p.opener_fails = True
        p.hold('q', 1.0)                           # key still held while unplugged
        self.assertEqual(p.diag.link_state, LINK_DISCONNECTED)
        self.assertTrue(any('I/O error' in t for t in p.bus.log_text('error')))
        self.assertEqual(p.fake.model.applied[0], 0.0)           # firmware watchdog (sim) stopped it
        p.opener_fails = False                                     # re-plug
        p.hold('q', 3.0)                                           # key STILL held
        self.assertEqual(p.diag.link_state, LINK_CONNECTED)
        self.assertEqual(p.diag.reconnect_count, 1)
        self.assertEqual(p.fake.model.applied[0], 0.0)           # no automatic resume
        p.advance(1.0)
        p.hold('q', 0.4)
        self.assertEqual(p.fake.model.applied[0], 0.1)           # new press works

    def test_hardware_mode_never_falls_back_to_mock(self):
        bus = fake_ros.new_bus()
        for name in ('hardware',):
            bus.load_yaml(CONFIG / f'{name}.yaml')

        def failing():
            raise OSError('no such device')
        with self.assertRaises(RuntimeError):
            SerialArmBridge(opener=failing)
        self.assertTrue(any('cannot open serial port' in t for t in bus.log_text('error')))
        self.assertFalse(bus.messages('/arm/state'))             # no simulated data appears

    def test_keep_retrying_mode_blocks_motion_and_reports_disconnected(self):
        bus = fake_ros.new_bus()
        bus.load_yaml(CONFIG / 'hardware.yaml')
        bus.overrides['serial_arm_bridge']['exit_on_initial_open_failure'] = False

        def failing():
            raise OSError('no such device')
        node = SerialArmBridge(opener=failing)
        bus.advance(2.5)
        self.assertEqual(bus.messages('/arm/diagnostics')[-1].link_state, LINK_DISCONNECTED)
        self.assertEqual(len([t for t in bus.log_text('error') if 'cannot open' in t]), 1)
        node.destroy_node()

    def test_shutdown_sends_stop_and_closes_port(self):
        p = Pipeline('serial')
        p.warm_up()
        port = p.port
        p.bridge.destroy_node()
        self.assertTrue(port.closed)
        self.assertEqual(p.fake.model.applied, [0.0] * 7)


if __name__ == '__main__':
    unittest.main()
