"""Serial bridge core, mock bridge core, firmware simulator and fake-STM32 tool (no ROS needed)."""
import math
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

_SRC = Path(__file__).resolve().parents[2]
for _p in ('arm_control', 'arm_hardware'):
    sys.path.insert(0, str(_SRC / _p))

from arm_hardware import serial_protocol as sp  # noqa: E402
from arm_hardware import telemetry_parser as tp  # noqa: E402
from arm_hardware.bridge_types import (LINK_CONNECTED, LINK_CONNECTING, LINK_DISCONNECTED,  # noqa: E402
                                       LINK_PROTOCOL_MISMATCH, LINK_TELEMETRY_STALE)
from arm_hardware.command_gate import MotionGate  # noqa: E402
from arm_hardware.fake_stm32 import FakeStm32  # noqa: E402
from arm_hardware.firmware_sim import FirmwareSimModel  # noqa: E402
from arm_hardware.mock_bridge_core import MockBridgeCore  # noqa: E402
from arm_hardware.serial_bridge_core import BridgeConfig, SerialBridgeCore  # noqa: E402

SET, STOP, ESTOP, REARM = 1, 2, 3, 4


def motors(o=None):
    ms = [NS(channel=c, direction=0, demand=0.0) for c in range(1, 8)]
    for c, (d, x) in (o or {}).items():
        ms[c - 1] = NS(channel=c, direction=d, demand=x)
    return ms


def cmd(ctype=SET, seq=0, o=None):
    return NS(command_type=ctype, sequence=seq, motors=motors(o))


MOVE = {1: (1, 0.2)}


class FakePort:
    def __init__(self, rig):
        self.rig, self.rx, self.closed = rig, bytearray(), False
        self.written, self.fail_read, self.fail_write = [], False, False

    def write(self, data):
        if self.fail_write:
            raise OSError('write failed')
        self.written.append(bytes(data))
        self.rig.fake.receive(bytes(data), self.rig.t)

    def read(self, n):
        if self.fail_read:
            raise OSError('device disconnected')
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def close(self):
        self.closed = True


class Rig:
    """Serial bridge core wired to a FakeStm32 through an in-memory port; time is manual."""

    def __init__(self, port_available=True, stale=0.5, reconnect=1.0, **fake_kw):
        self.t = 0.0
        self.fake = FakeStm32(**fake_kw)
        self.ports = []
        self.port_available = port_available
        self.core = SerialBridgeCore(self._open, BridgeConfig(telemetry_stale_after_s=stale,
                                                              reconnect_interval_s=reconnect))
        self.telemetry = []

    def _open(self):
        if not self.port_available:
            raise OSError('no such device')
        p = FakePort(self)
        self.ports.append(p)
        return p

    @property
    def port(self):
        return self.ports[-1]

    def run(self, seconds, feed=True, dt=0.05):
        end = self.t + seconds
        while self.t < end - 1e-9:
            self.t = round(self.t + dt, 6)
            if feed and self.core.port_open:
                self.port.rx += self.fake.telemetry_frame(self.t)
            self.telemetry += self.core.poll(self.t)

    def connect(self):
        self.core.open(self.t)
        self.run(0.2)
        assert self.core.link_state == LINK_CONNECTED, self.core.link_state

    def send(self, c):
        return self.core.on_command(c, self.t)


class TestFirmwareSim(unittest.TestCase):
    def setUp(self):
        self.m = FirmwareSimModel(command_timeout_s=0.5)
        self.m.step(0.0)
        self.m.step(0.05)

    def dec(self, ctype, seq, permille=None):
        t = {SET: sp.TYPE_SETPOINT, STOP: sp.TYPE_STOP, ESTOP: sp.TYPE_ESTOP, REARM: sp.TYPE_REARM}[ctype]
        return sp.DecodedCommand(t, seq, permille)

    def test_boot_to_idle_and_outputs_zero(self):
        self.assertEqual(self.m.fw_state, 2)
        self.assertEqual(self.m.applied, [0.0] * 7)

    def test_setpoint_accepted_and_echoed(self):
        self.assertTrue(self.m.handle_command(self.dec(SET, 41, [200, 0, 0, 0, 0, 0, 0]), 0.1))
        self.m.step(0.1)
        t = self.m.telemetry(0.1)
        self.assertEqual((t.accepted_seq_valid, t.last_accepted_cmd_seq, t.firmware_state), (True, 41, 3))
        self.assertAlmostEqual(t.channels[0].applied_output, 0.2)

    def test_estop_latches_rejects_setpoints_until_rearm_and_never_resumes(self):
        self.m.handle_command(self.dec(SET, 1, [300] * 7), 0.1)
        self.m.handle_command(self.dec(ESTOP, 2), 0.12)
        self.m.step(0.13)
        self.assertEqual(self.m.applied, [0.0] * 7)
        self.assertEqual(self.m.telemetry(0.13).latched_faults & 2, 2)
        self.assertFalse(self.m.handle_command(self.dec(SET, 3, [300] * 7), 0.14))
        self.assertEqual(self.m.telemetry(0.14).last_accepted_cmd_seq, 2)   # not accepted
        self.assertTrue(self.m.handle_command(self.dec(REARM, 4), 0.15))
        self.m.step(0.16)
        t = self.m.telemetry(0.16)
        self.assertEqual((t.latched_faults, t.channels[0].applied_output), (0, 0.0))   # still no motion
        self.m.step(0.2)
        self.assertEqual(self.m.applied, [0.0] * 7)

    def test_command_timeout_zeroes_outputs_and_reports(self):
        self.m.handle_command(self.dec(SET, 1, [300] * 7), 0.1)
        self.m.step(0.2)
        self.assertTrue(any(self.m.applied))
        self.m.step(0.7)                         # > 0.5 s since last command
        t = self.m.telemetry(0.7)
        self.assertEqual(self.m.applied, [0.0] * 7)
        self.assertTrue(t.active_faults & 1)
        self.m.handle_command(self.dec(SET, 2, [0] * 7), 0.8)
        self.assertFalse(self.m.telemetry(0.8).active_faults & 1)

    def test_timeout_does_not_resume_motion_when_link_returns(self):
        self.m.handle_command(self.dec(SET, 1, [300] * 7), 0.1)
        self.m.step(0.7)
        self.m.step(0.8)
        self.assertEqual(self.m.applied, [0.0] * 7)      # targets were cleared, not restored

    def test_invalid_sensor_never_reads_as_zero(self):
        self.m.invalid_channels = {2}
        t = self.m.telemetry(0.1)
        self.assertEqual(t.channels[1].valid_flags & 4, 0)
        self.assertEqual(t.channels[1].fault_flags, 1)
        self.assertTrue(t.active_faults & 4)

    def test_sensor_types_per_channel(self):
        t = self.m.telemetry(0.1)
        kinds = [(c.valid_flags & 2 > 0, c.valid_flags & 4 > 0, c.valid_flags & 8 > 0) for c in t.channels]
        self.assertEqual(kinds, [(True, False, False), (False, True, False), (False, True, False),
                                 (True, False, False), (True, False, False), (True, False, False),
                                 (False, False, True)])


class TestGate(unittest.TestCase):
    def test_rules(self):
        g = MotionGate()
        self.assertTrue(g.decide(cmd(SET, o=MOVE), True).substitute_zero)       # closed at start
        self.assertFalse(g.decide(cmd(SET), True).substitute_zero)              # zero opens it
        self.assertTrue(g.open)
        self.assertTrue(g.decide(cmd(SET, o=MOVE), True).transmit_as_is)
        g.close()
        self.assertTrue(g.decide(cmd(SET, o=MOVE), True).substitute_zero)
        self.assertTrue(g.decide(cmd(ESTOP), False).transmit_as_is)             # stops always pass
        self.assertTrue(g.decide(cmd(STOP), False).transmit_as_is)
        self.assertFalse(g.open)                                                # unhealthy: stays closed
        self.assertTrue(g.decide(cmd(SET, o=MOVE), False).substitute_zero)
        g.decide(cmd(REARM), True)
        self.assertFalse(g.open)                                                # REARM does not open it


class TestSerialBridge(unittest.TestCase):
    def test_connect_and_telemetry_decoded(self):
        r = Rig()
        r.core.open(r.t)
        self.assertEqual(r.core.link_state, LINK_CONNECTING)
        r.run(0.2)
        self.assertEqual(r.core.link_state, LINK_CONNECTED)
        self.assertGreater(len(r.telemetry), 2)
        self.assertGreater(r.core.stats.frames_received_valid, 2)

    def test_command_transmitted_then_acceptance_comes_back_in_telemetry(self):
        r = Rig()
        r.connect()
        self.assertEqual(r.send(cmd(SET, 100)), 'sent')            # zero opens the gate
        self.assertEqual(r.send(cmd(SET, 101, MOVE)), 'sent')
        r.run(0.1)
        self.assertEqual(r.telemetry[-1].last_accepted_cmd_seq, 101)
        self.assertTrue(r.telemetry[-1].accepted_seq_valid)
        self.assertAlmostEqual(r.telemetry[-1].channels[0].applied_output, 0.2)
        self.assertEqual(r.core.stats.frames_transmitted, 2)

    def test_transmitted_is_not_accepted(self):
        r = Rig()
        r.connect()
        r.send(cmd(SET, 7))
        # Nothing run yet: no telemetry has confirmed seq 7, bridge only counts "transmitted".
        self.assertNotEqual(r.telemetry[-1].last_accepted_cmd_seq, 7)

    def test_no_motion_until_zero_seen_after_connect(self):
        r = Rig()
        r.connect()
        self.assertEqual(r.send(cmd(SET, 1, MOVE)), 'sent_zero_substituted')
        r.run(0.1)
        self.assertEqual(r.telemetry[-1].channels[0].applied_output, 0.0)
        self.assertEqual(r.send(cmd(SET, 2)), 'sent')
        self.assertEqual(r.send(cmd(SET, 3, MOVE)), 'sent')
        self.assertEqual(r.core.stats.commands_rejected, 1)

    def test_invalid_commands_rejected_and_never_written(self):
        r = Rig()
        r.connect()
        written = len(r.port.written)
        for bad in (cmd(0), cmd(SET, o={1: (1, math.nan)}), NS(command_type=SET, sequence=0, motors=[]),
                    cmd(SET, o={1: (1, 2.0)})):
            self.assertEqual(r.send(bad), 'rejected_invalid')
        self.assertEqual(len(r.port.written), written)
        self.assertEqual(r.core.stats.commands_rejected, 4)

    def test_estop_always_transmitted_even_when_stale(self):
        r = Rig()
        r.connect()
        r.run(1.0, feed=False)
        self.assertEqual(r.core.link_state, LINK_TELEMETRY_STALE)
        self.assertEqual(r.send(cmd(ESTOP, 9, None)), 'sent')
        (f,) = sp.FrameDecoder().feed(r.port.written[-1])
        self.assertEqual(f.msg_type, sp.TYPE_ESTOP)

    def test_stale_telemetry_detected_and_recovery_requires_zero_before_motion(self):
        r = Rig(stale=0.5)
        r.connect()
        r.send(cmd(SET, 1))
        self.assertEqual(r.send(cmd(SET, 2, MOVE)), 'sent')
        r.run(0.8, feed=False)
        self.assertEqual(r.core.link_state, LINK_TELEMETRY_STALE)
        self.assertEqual(r.core.stats.telemetry_timeouts, 1)
        self.assertGreater(r.core.stats.last_telemetry_age_s, 0.5)
        r.run(0.2)                                                  # telemetry returns
        self.assertEqual(r.core.link_state, LINK_CONNECTED)
        self.assertEqual(r.send(cmd(SET, 3, MOVE)), 'sent_zero_substituted')   # no auto resume
        self.assertEqual(r.send(cmd(SET, 4)), 'sent')
        self.assertEqual(r.send(cmd(SET, 5, MOVE)), 'sent')

    def test_port_never_opened_reports_disconnected_and_logs_once(self):
        r = Rig(port_available=False)
        for _ in range(5):
            r.run(1.1)
        self.assertEqual(r.core.link_state, LINK_DISCONNECTED)
        errors = [e for e in r.core.drain_events() if e[0] == 'error']
        self.assertEqual(len(errors), 1)            # not spammed on every retry
        self.assertEqual(r.send(cmd(SET)), 'not_connected')

    def test_unplug_then_reconnect_does_not_resume_motion(self):
        r = Rig(reconnect=1.0)
        r.connect()
        r.send(cmd(SET, 1))
        r.send(cmd(SET, 2, MOVE))
        r.port.fail_read = True                     # cable pulled
        r.run(0.1)
        self.assertEqual(r.core.link_state, LINK_DISCONNECTED)
        self.assertTrue(r.ports[0].closed)
        self.assertEqual(r.send(cmd(SET, 3, MOVE)), 'not_connected')
        r.run(1.2)                                  # reconnect attempt succeeds
        self.assertEqual(r.core.stats.reconnect_count, 1)
        r.run(0.2)
        self.assertEqual(r.core.link_state, LINK_CONNECTED)
        self.assertEqual(r.send(cmd(SET, 4, MOVE)), 'sent_zero_substituted')
        self.assertEqual(r.send(cmd(SET, 5)), 'sent')
        self.assertEqual(r.send(cmd(SET, 6, MOVE)), 'sent')

    def test_write_failure_closes_port(self):
        r = Rig()
        r.connect()
        r.port.fail_write = True
        self.assertEqual(r.send(cmd(SET, 1)), 'not_connected')
        self.assertEqual(r.core.link_state, LINK_DISCONNECTED)

    def test_corrupt_and_garbage_data_counted_not_fatal(self):
        r = Rig(corrupt_rate=0.5)
        r.connect()
        r.port.rx += b'\x11\x22\x33\x00' + bytes(range(1, 200)) + b'\x00'
        r.run(3.0)
        s = r.core.stats
        self.assertGreater(s.frames_crc_error + s.frames_malformed, 3)
        self.assertGreater(s.frames_received_valid, 3)
        self.assertEqual(r.core.link_state, LINK_CONNECTED)

    def test_partial_frames_across_reads(self):
        r = Rig()
        r.core.open(r.t)
        frame = r.fake.telemetry_frame(0.05)
        for b in frame:                              # one byte per poll
            r.t += 0.001
            r.port.rx += bytes([b])
            r.telemetry += r.core.poll(r.t)
        self.assertEqual(len(r.telemetry), 1)

    def test_protocol_version_mismatch_blocks_setpoints_but_not_stops(self):
        r = Rig(version=2)
        r.core.open(r.t)
        r.run(0.3)
        self.assertEqual(r.core.link_state, LINK_PROTOCOL_MISMATCH)
        self.assertEqual((r.core.stats.remote_version_valid, r.core.stats.remote_protocol_version), (True, 2))
        self.assertEqual(r.send(cmd(SET, 1)), 'blocked_mismatch')
        self.assertEqual(r.send(cmd(STOP, 2, None)), 'sent')
        self.assertEqual(r.send(cmd(ESTOP, 3, None)), 'sent')
        self.assertEqual(r.telemetry, [])            # mismatching telemetry is not published

    def test_close_sends_stop_and_is_idempotent(self):
        r = Rig()
        r.connect()
        port = r.port
        r.core.close()
        self.assertTrue(port.closed)
        (f,) = sp.FrameDecoder().feed(port.written[-1])
        self.assertEqual(f.msg_type, sp.TYPE_STOP)
        r.core.close()
        self.assertEqual(r.core.link_state, LINK_DISCONNECTED)

    def test_poll_is_bounded_work(self):
        r = Rig()
        r.connect()
        r.port.rx += b'\x55' * 100000                # flood
        r.core.poll(r.t + 0.01)
        self.assertGreater(len(r.port.rx), 0)        # did not try to drain everything in one poll


class TestMockBridgeCore(unittest.TestCase):
    def test_everything_is_marked_simulated(self):
        c = MockBridgeCore()
        c.tick(0.0)
        self.assertTrue(c.stats.simulated)

    def test_command_path_uses_real_protocol_and_gate(self):
        c = MockBridgeCore()
        c.tick(0.0)
        c.tick(0.05)
        self.assertEqual(c.on_command(cmd(SET, 1, MOVE), 0.06), 'gated_zero')
        self.assertEqual(c.on_command(cmd(SET, 2), 0.07), 'delivered')
        self.assertEqual(c.on_command(cmd(SET, 3, MOVE), 0.08), 'delivered')
        t = c.tick(0.1)
        self.assertAlmostEqual(t.channels[0].applied_output, 0.2)
        self.assertEqual((t.accepted_seq_valid, t.last_accepted_cmd_seq), (True, 3))

    def test_invalid_command_rejected(self):
        c = MockBridgeCore()
        self.assertEqual(c.on_command(cmd(SET, 1, {1: (1, math.inf)}), 0.0), 'rejected_invalid')
        self.assertEqual(c.stats.commands_rejected, 1)

    def test_all_seven_channels_map(self):
        for ch in range(1, 8):
            c = MockBridgeCore()
            c.tick(0.0)
            c.on_command(cmd(SET, 1), 0.01)
            c.on_command(cmd(SET, 2, {ch: (-1, 0.3)}), 0.02)
            t = c.tick(0.05)
            outs = [x.applied_output for x in t.channels]
            self.assertAlmostEqual(outs[ch - 1], -0.3)
            self.assertEqual(sum(1 for o in outs if o), 1)

    def test_stop_and_estop_propagate(self):
        c = MockBridgeCore()
        c.tick(0.0)
        c.on_command(cmd(SET, 1), 0.01)
        c.on_command(cmd(SET, 2, MOVE), 0.02)
        c.on_command(cmd(STOP, 3, None), 0.03)
        self.assertEqual(c.tick(0.04).channels[0].applied_output, 0.0)
        c.on_command(cmd(SET, 4, MOVE), 0.05)
        c.on_command(cmd(ESTOP, 5, None), 0.06)
        t = c.tick(0.07)
        self.assertEqual((t.channels[0].applied_output, t.active_faults & 2), (0.0, 2))
        self.assertEqual(c.stats.latched_faults, 2)

    def test_comm_loss_stops_telemetry_marks_stale_and_blocks_resume(self):
        c = MockBridgeCore(command_timeout_s=0.5, telemetry_stale_after_s=0.5)
        c.tick(0.0)
        c.on_command(cmd(SET, 1), 0.01)
        c.on_command(cmd(SET, 2, MOVE), 0.02)
        c.tick(0.05)
        c.set_comm_loss(True, 0.1)
        self.assertIsNone(c.tick(0.2))
        self.assertIsNone(c.tick(0.9))
        self.assertEqual(c.stats.link_state, LINK_TELEMETRY_STALE)
        self.assertEqual(c.stats.telemetry_timeouts, 1)
        c.set_comm_loss(False, 1.0)
        t = c.tick(1.05)
        self.assertTrue(t.active_faults & 1)                     # firmware reports timeout
        self.assertEqual(t.channels[0].applied_output, 0.0)
        self.assertEqual(c.on_command(cmd(SET, 3, MOVE), 1.06), 'gated_zero')   # no auto resume

    def test_invalid_sensor_injection(self):
        c = MockBridgeCore()
        c.set_invalid_sensor_channel(3)
        t = c.tick(0.0)
        self.assertEqual(t.channels[2].valid_flags & 4, 0)
        self.assertTrue(math.isnan(t.channels[2].joint_angle_rad))

    def test_restart_is_predictable(self):
        a, b = MockBridgeCore(), MockBridgeCore()
        for c in (a, b):
            c.tick(0.0)
            c.on_command(cmd(SET, 1), 0.01)
            c.on_command(cmd(SET, 2, MOVE), 0.02)
        self.assertEqual(a.tick(0.05).channels[0].applied_output, b.tick(0.05).channels[0].applied_output)
        fresh = MockBridgeCore()
        self.assertEqual(fresh.tick(0.0).channels[0].applied_output, 0.0)   # restart starts at rest


@unittest.skipUnless(hasattr(os, 'openpty'), 'needs a POSIX pseudo-terminal')
class TestFakeStm32Tool(unittest.TestCase):
    def test_cli_speaks_the_protocol_over_a_real_pty(self):
        import select
        import tty
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(str(_SRC / p) for p in ('arm_hardware', 'arm_control')))
        proc = subprocess.Popen([sys.executable, '-u', '-m', 'arm_hardware.fake_stm32', '--rate', '50'],
                                stdout=subprocess.PIPE, text=True, env=env)
        try:
            line = proc.stdout.readline()
            path = line.strip().split('slave port: ')[1]
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(fd)
            os.write(fd, sp.encode_command(cmd(SET, 77)))
            dec, frames, deadline = sp.FrameDecoder(), [], time.monotonic() + 3.0
            while time.monotonic() < deadline and len(frames) < 10:
                if select.select([fd], [], [], 0.1)[0]:
                    frames += [x for x in dec.feed(os.read(fd, 1024)) if isinstance(x, sp.Frame)]
            os.close(fd)
            self.assertGreaterEqual(len(frames), 10)
            data = [tp.parse_telemetry_frame(f) for f in frames]
            self.assertTrue(any(d.accepted_seq_valid and d.last_accepted_cmd_seq == 77 for d in data))
        finally:
            proc.terminate()
            proc.wait(timeout=5)
            proc.stdout.close()


if __name__ == '__main__':
    unittest.main()
