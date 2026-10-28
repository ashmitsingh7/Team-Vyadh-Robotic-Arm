"""ManualControlCore, command mapper, software limits, config and state assessment."""
import math
import unittest
from types import SimpleNamespace as NS

from arm_control.arm_state_monitor import StateAssessment, assess, format_summary
from arm_control.command_mapper import MappingError, map_move
from arm_control.command_validation import validate_arm_command
from arm_control.control_config import (ChannelConfig, ControlConfig, validate_config)
from arm_control.manual_control_core import ManualControlCore
from arm_control.software_limits import check_motion

SET, STOP, ESTOP, REARM = 1, 2, 3, 4
OK = StateAssessment(fresh=True, age_s=0.0, firmware_state=2, link_state=2)   # healthy
BLOCKED = StateAssessment(blocked_reason='telemetry stale (1.00 s old)')


def mv(channel, direction, level=1.0):
    return NS(action=1, channel=channel, direction=direction, level=level)


def act(action):
    return NS(action=action, channel=0, direction=0, level=0.0)


def targets(cmd):
    return {m.channel: (m.direction, m.demand) for m in cmd.motors if m.direction}


def make(**cfg_kw):
    cfg = ControlConfig(**cfg_kw)
    return ManualControlCore(cfg), cfg


class TestMapper(unittest.TestCase):
    def test_polarity_and_max_demand(self):
        c = ChannelConfig(polarity=-1, max_demand=0.4)
        self.assertEqual(map_move(1, 1, 0.5, c).direction, -1)
        self.assertAlmostEqual(map_move(1, 1, 0.5, c).demand, 0.2)
        self.assertEqual(map_move(1, -1, 1.0, c).direction, 1)

    def test_channels_independent_polarity(self):
        a, b = ChannelConfig(polarity=1), ChannelConfig(polarity=-1)
        self.assertNotEqual(map_move(1, 1, 1.0, a).direction, map_move(2, 1, 1.0, b).direction)

    def test_disabled_channel_rejected_but_release_allowed(self):
        c = ChannelConfig(enabled=False)
        with self.assertRaises(MappingError):
            map_move(1, 1, 1.0, c)
        self.assertEqual(map_move(1, 0, 0.0, c).direction, 0)


class TestLimits(unittest.TestCase):
    def test_disabled_limits_pass(self):
        self.assertIsNone(check_motion(2, 1, ChannelConfig(), None))

    def test_zero_direction_never_blocked(self):
        c = ChannelConfig(joint_limits_enabled=True, joint_min_rad=0, joint_max_rad=1)
        self.assertIsNone(check_motion(2, 0, c, None))

    def test_limit_directions_and_sign(self):
        c = ChannelConfig(joint_limits_enabled=True, joint_min_rad=0.0, joint_max_rad=1.0)
        self.assertIsNone(check_motion(2, 1, c, 0.5))
        self.assertTrue(check_motion(2, 1, c, 1.0))      # at max moving up
        self.assertIsNone(check_motion(2, -1, c, 1.0))   # at max moving down is allowed
        self.assertTrue(check_motion(2, -1, c, 0.0))
        self.assertIsNone(check_motion(2, 1, c, 0.0))
        c.angle_sign = -1                                 # +direction now decreases the angle
        self.assertTrue(check_motion(2, 1, c, 0.0))
        self.assertIsNone(check_motion(2, 1, c, 1.0))

    def test_unavailable_angle_blocks_when_limits_enabled(self):
        c = ChannelConfig(joint_limits_enabled=True, joint_min_rad=0, joint_max_rad=1)
        for bad in (None, math.nan, math.inf):
            self.assertTrue(check_motion(2, 1, c, bad))

    def test_disabled_channel(self):
        self.assertTrue(check_motion(1, 1, ChannelConfig(enabled=False), None))


class TestConfig(unittest.TestCase):
    def test_defaults_valid(self):
        self.assertEqual(validate_config(ControlConfig()), [])

    def test_bad_values(self):
        cfg = ControlConfig()
        cfg.channels[1].polarity = 0
        cfg.channels[2].max_demand = 1.5
        cfg.channels[3].joint_limits_enabled = True       # min == max == 0
        cfg.input_timeout_s = 0.01
        problems = validate_config(cfg)
        self.assertEqual(len(problems), 4, problems)


class TestCoreMotion(unittest.TestCase):
    def test_idle_publishes_full_zero_snapshot(self):
        core, _ = make()
        c = core.tick(0.0, OK)
        self.assertEqual((c.command_type, len(c.motors)), (SET, 7))
        self.assertEqual(targets(c), {})
        self.assertEqual(validate_arm_command(c), [])

    def test_each_of_seven_channels_both_directions(self):
        for ch in range(1, 8):
            for d in (-1, 1):
                core, cfg = make()
                self.assertTrue(core.handle_user_input(mv(ch, d, 1.0), 0.0, OK).accepted)
                c = core.tick(0.01, OK)
                self.assertEqual(targets(c), {ch: (d, cfg.channels[ch].max_demand)}, (ch, d))

    def test_simultaneous_channels_independent(self):
        core, _ = make()
        core.handle_user_input(mv(1, 1), 0.0, OK)
        core.handle_user_input(mv(5, -1), 0.0, OK)
        core.handle_user_input(mv(6, 1), 0.0, OK)
        c = core.tick(0.01, OK)
        self.assertEqual(sorted(targets(c)), [1, 5, 6])   # M5/M6 independently addressable

    def test_release_only_affects_that_channel(self):
        core, _ = make()
        core.handle_user_input(mv(1, 1), 0.0, OK)
        core.handle_user_input(mv(2, 1), 0.0, OK)
        core.handle_user_input(mv(1, 0, 0.0), 0.01, OK)
        self.assertEqual(sorted(targets(core.tick(0.02, OK))), [2])

    def test_sequence_increments_and_wraps(self):
        core, _ = make()
        seqs = [core.tick(i * 0.05, OK).sequence for i in range(3)]
        self.assertEqual(seqs, [0, 1, 2])
        core._seq = 0xFFFFFFFF
        self.assertEqual(core.tick(1.0, OK).sequence, 0xFFFFFFFF)
        self.assertEqual(core.tick(1.1, OK).sequence, 0)

    def test_input_expires_without_refresh(self):
        core, cfg = make()
        core.handle_user_input(mv(3, 1), 0.0, OK)
        self.assertTrue(targets(core.tick(cfg.input_timeout_s - 0.01, OK)))
        self.assertFalse(targets(core.tick(cfg.input_timeout_s + 0.01, OK)))

    def test_refresh_extends(self):
        core, cfg = make()
        core.handle_user_input(mv(3, 1), 0.0, OK)
        core.handle_user_input(mv(3, 1), 0.25, OK)
        self.assertTrue(targets(core.tick(0.4, OK)))

    def test_direction_reversal_replaces_target(self):
        core, _ = make()
        core.handle_user_input(mv(3, 1), 0.0, OK)
        core.handle_user_input(mv(3, -1), 0.05, OK)
        self.assertEqual(targets(core.tick(0.06, OK))[3][0], -1)

    def test_invalid_inputs_cause_no_motion(self):
        core, _ = make()
        for bad in (mv(0, 1), mv(8, 1), mv(1, 2), mv(1, 1, math.nan), mv(1, 1, 2.0),
                    NS(action=0, channel=1, direction=1, level=0.5), mv(1, 1, 0.0)):
            self.assertFalse(core.handle_user_input(bad, 0.0, OK).accepted)
        self.assertFalse(targets(core.tick(0.01, OK)))
        self.assertEqual(core.counters.inputs_rejected, 7)

    def test_disabled_channel_cannot_move(self):
        core, cfg = make()
        cfg.channels[4].enabled = False
        self.assertFalse(core.handle_user_input(mv(4, 1), 0.0, OK).accepted)
        self.assertFalse(targets(core.tick(0.01, OK)))


class TestCoreStops(unittest.TestCase):
    def test_stop_clears_all_and_publishes_stop_type(self):
        core, cfg = make()
        core.handle_user_input(mv(1, 1), 0.0, OK)
        core.handle_user_input(mv(2, 1), 0.0, OK)
        self.assertTrue(core.handle_user_input(act(2), 0.01, OK).accepted)
        types = [core.tick(0.02 + i * 0.05, OK).command_type for i in range(cfg.stop_repeat + 2)]
        self.assertEqual(types[:cfg.stop_repeat], [STOP] * cfg.stop_repeat)
        self.assertEqual(types[cfg.stop_repeat:], [SET, SET])
        self.assertFalse(core.estop_latched)   # normal stop is not latched

    def test_move_after_stop_cancels_pending_stop_type(self):
        core, _ = make()
        core.handle_user_input(act(2), 0.0, OK)
        core.handle_user_input(mv(1, 1), 0.01, OK)
        self.assertEqual(core.tick(0.02, OK).command_type, SET)

    def test_estop_latches_and_blocks_motion(self):
        core, _ = make()
        core.handle_user_input(mv(1, 1), 0.0, OK)
        core.handle_user_input(act(3), 0.01, OK)
        for i in range(5):
            c = core.tick(0.02 + i * 0.05, OK)
            self.assertEqual(c.command_type, ESTOP)
            self.assertFalse(targets(c))
        self.assertFalse(core.handle_user_input(mv(1, 1), 1.0, OK).accepted)
        self.assertEqual(core.tick(1.01, OK).command_type, ESTOP)

    def test_estop_not_cleared_by_healthy_state_or_time(self):
        core, _ = make()
        core.handle_user_input(act(3), 0.0, OK)
        for t in (10.0, 100.0, 1000.0):
            self.assertEqual(core.tick(t, OK).command_type, ESTOP)

    def test_stop_never_clears_estop(self):
        core, _ = make()
        core.handle_user_input(act(3), 0.0, OK)
        core.handle_user_input(act(2), 0.1, OK)
        self.assertEqual(core.tick(0.2, OK).command_type, ESTOP)

    def test_rearm_sequence_then_no_motion_until_new_move(self):
        core, cfg = make()
        core.handle_user_input(mv(1, 1), 0.0, OK)
        core.handle_user_input(act(3), 0.01, OK)
        self.assertTrue(core.handle_user_input(act(4), 0.5, OK).accepted)
        types = []
        for i in range(cfg.rearm_repeat + 2):
            c = core.tick(0.6 + i * 0.05, OK)
            types.append(c.command_type)
            self.assertFalse(targets(c))          # nothing moves during/after REARM
        self.assertEqual(types, [REARM] * cfg.rearm_repeat + [SET, SET])
        # held key from before ESTOP is gone; a NEW move is required
        self.assertTrue(core.handle_user_input(mv(1, 1), 2.0, OK).accepted)
        self.assertTrue(targets(core.tick(2.01, OK)))

    def test_move_rejected_during_rearm(self):
        core, _ = make()
        core.handle_user_input(act(3), 0.0, OK)
        core.handle_user_input(act(4), 0.1, OK)
        self.assertFalse(core.handle_user_input(mv(1, 1), 0.11, OK).accepted)

    def test_rearm_without_estop_is_rejected_and_harmless(self):
        core, _ = make()
        self.assertFalse(core.handle_user_input(act(4), 0.0, OK).accepted)
        self.assertEqual(core.tick(0.1, OK).command_type, SET)

    def test_estop_after_rearm_relatches(self):
        core, _ = make()
        core.handle_user_input(act(3), 0.0, OK)
        core.handle_user_input(act(4), 0.1, OK)
        core.handle_user_input(act(3), 0.12, OK)
        self.assertEqual(core.tick(0.2, OK).command_type, ESTOP)

    def test_estop_honoured_even_with_garbage_fields_and_bad_state(self):
        core, _ = make()
        core.handle_user_input(NS(action=3, channel=99, direction=9, level=math.nan), 0.0, BLOCKED)
        self.assertTrue(core.estop_latched)


class TestCoreStateGating(unittest.TestCase):
    def test_no_motion_when_state_blocked(self):
        core, _ = make()
        res = core.handle_user_input(mv(1, 1), 0.0, BLOCKED)
        self.assertFalse(res.accepted)
        self.assertIn('stale', res.reason)
        self.assertFalse(targets(core.tick(0.01, BLOCKED)))

    def test_active_motion_dropped_when_state_goes_stale(self):
        core, _ = make()
        core.handle_user_input(mv(1, 1), 0.0, OK)
        self.assertTrue(targets(core.tick(0.01, OK)))
        self.assertFalse(targets(core.tick(0.02, BLOCKED)))
        # and it does not come back by itself when state recovers (needs a fresh operator MOVE)
        self.assertFalse(targets(core.tick(0.03, OK)))

    def test_release_allowed_when_blocked(self):
        core, _ = make()
        self.assertTrue(core.handle_user_input(mv(1, 0, 0.0), 0.0, BLOCKED).accepted)

    def test_gating_can_be_disabled_for_bringup(self):
        core, _ = make(require_fresh_state=False)
        self.assertTrue(core.handle_user_input(mv(1, 1), 0.0, BLOCKED).accepted)

    def test_joint_limit_blocks_and_drops(self):
        core, cfg = make()
        cfg.channels[2] = ChannelConfig(joint_limits_enabled=True, joint_min_rad=0.0, joint_max_rad=1.0)
        a = StateAssessment(fresh=True, firmware_state=3, link_state=2, angles={2: 0.5})
        self.assertTrue(core.handle_user_input(mv(2, 1), 0.0, a).accepted)
        a2 = StateAssessment(fresh=True, firmware_state=3, link_state=2, angles={2: 1.0})
        self.assertFalse(targets(core.tick(0.01, a2)))                 # dropped at the limit
        self.assertFalse(core.handle_user_input(mv(2, 1), 0.02, a2).accepted)
        self.assertTrue(core.handle_user_input(mv(2, -1), 0.03, a2).accepted)   # back off allowed

    def test_joint_limit_missing_angle_blocks(self):
        core, cfg = make()
        cfg.channels[3] = ChannelConfig(joint_limits_enabled=True, joint_min_rad=0.0, joint_max_rad=1.0)
        a = StateAssessment(fresh=True, firmware_state=3, link_state=2, angles={3: None})
        self.assertFalse(core.handle_user_input(mv(3, 1), 0.0, a).accepted)

    def test_every_published_command_validates(self):
        core, _ = make()
        for i in range(50):
            if i % 7 == 0:
                core.handle_user_input(mv(1 + i % 7, 1 if i % 2 else -1, 0.3), i * 0.05, OK)
            if i == 20:
                core.handle_user_input(act(3), i * 0.05, OK)
            if i == 30:
                core.handle_user_input(act(4), i * 0.05, OK)
            self.assertEqual(validate_arm_command(core.tick(i * 0.05, OK)), [])


class TestNoAutoResume(unittest.TestCase):
    """A key still held through a link loss or ESTOP must not restart motion by itself."""

    def hold(self, core, ch, t0, t1, a, step=0.05):
        """Operator keeps sending MOVE for ch every `step` seconds from t0 to t1; returns last tick."""
        t, last = t0, None
        while t < t1:
            core.handle_user_input(mv(ch, 1), t, a)
            last = core.tick(t, a)
            t = round(t + step, 6)
        return last

    def test_held_key_does_not_resume_after_state_recovers(self):
        core, _ = make()
        self.assertTrue(targets(self.hold(core, 1, 0.0, 0.5, OK)))
        self.assertFalse(targets(self.hold(core, 1, 0.5, 1.5, BLOCKED)))   # link lost, key still held
        self.assertFalse(targets(self.hold(core, 1, 1.5, 3.0, OK)))        # recovered, key STILL held
        core.handle_user_input(mv(1, 0, 0.0), 3.1, OK)                      # operator releases
        self.assertTrue(core.handle_user_input(mv(1, 1), 3.2, OK).accepted)
        self.assertTrue(targets(core.tick(3.21, OK)))

    def test_holdoff_clears_when_input_goes_quiet(self):
        core, cfg = make()
        self.hold(core, 1, 0.0, 0.3, OK)
        self.hold(core, 1, 0.3, 0.6, BLOCKED)
        core.tick(0.6 + cfg.input_timeout_s + 0.1, OK)                      # key released, no explicit release
        self.assertTrue(core.handle_user_input(mv(1, 1), 1.2, OK).accepted)

    def test_held_key_does_not_resume_after_rearm(self):
        core, cfg = make()
        self.hold(core, 2, 0.0, 0.2, OK)
        core.handle_user_input(act(3), 0.2, OK)
        self.hold(core, 2, 0.25, 0.5, OK)                                   # held through ESTOP
        core.handle_user_input(act(4), 0.5, OK)
        last = self.hold(core, 2, 0.55, 2.0, OK)                            # held through REARM
        self.assertFalse(targets(last))
        self.assertTrue(core.handle_user_input(mv(3, 1), 2.1, OK).accepted)  # a different, new key is fine

    def test_other_channels_unaffected_by_holdoff(self):
        core, _ = make()
        core.handle_user_input(mv(1, 1), 0.0, BLOCKED)
        self.assertTrue(core.handle_user_input(mv(2, 1), 0.01, OK).accepted)

    def test_release_always_clears_holdoff(self):
        core, _ = make()
        core.handle_user_input(mv(1, 1), 0.0, BLOCKED)
        self.assertFalse(core.handle_user_input(mv(1, 1), 0.01, OK).accepted)
        core.handle_user_input(mv(1, 0, 0.0), 0.02, OK)
        self.assertTrue(core.handle_user_input(mv(1, 1), 0.03, OK).accepted)


class TestAssess(unittest.TestCase):
    def state(self, fw=3, faults=0, sim=False, angle=0.5):
        m = [NS(channel=c, valid_flags=4 if c == 2 else 0, joint_angle_rad=angle if c == 2 else math.nan,
                applied_output=0.0, encoder_counts=0, current_a=math.nan, fault_flags=0)
             for c in range(1, 8)]
        return NS(simulated=sim, firmware_state=fw, active_faults=faults, motors=m)

    def run_assess(self, state, rx, diag, drx, now=1.0):
        return assess(state, rx, diag, drx, now, 0.5, 5.0, 2 | 32 | 64)

    def test_healthy(self):
        a = self.run_assess(self.state(), 0.9, NS(link_state=2), 0.9)
        self.assertIsNone(a.blocked_reason)
        self.assertTrue(a.fresh)
        self.assertEqual(a.angles[2], 0.5)
        self.assertIsNone(a.angles[1])

    def test_none_stale_and_link_cases(self):
        self.assertIn('no telemetry', self.run_assess(None, None, None, None).blocked_reason)
        self.assertIn('stale', self.run_assess(self.state(), 0.0, NS(link_state=2), 0.9).blocked_reason)
        self.assertIn('diagnostics', self.run_assess(self.state(), 0.9, None, None).blocked_reason)
        self.assertIn('diagnostics', self.run_assess(self.state(), 0.9, NS(link_state=2), -10).blocked_reason)
        self.assertIn('TELEMETRY_STALE', self.run_assess(self.state(), 0.9, NS(link_state=3), 0.9).blocked_reason)

    def test_firmware_state_and_faults(self):
        d = NS(link_state=2)
        for fw in (0, 1, 5, 6):
            self.assertTrue(self.run_assess(self.state(fw=fw), 0.9, d, 0.9).blocked_reason, fw)
        for fw in (2, 3, 4):
            self.assertIsNone(self.run_assess(self.state(fw=fw), 0.9, d, 0.9).blocked_reason, fw)
        self.assertIn('ESTOP_LATCHED', self.run_assess(self.state(faults=2), 0.9, d, 0.9).blocked_reason)
        # command timeout fault is informational: must not deadlock recovery
        self.assertIsNone(self.run_assess(self.state(faults=1), 0.9, d, 0.9).blocked_reason)

    def test_summary_labels_simulated_and_stale(self):
        a = self.run_assess(self.state(sim=True), 0.0, NS(link_state=2), 0.9)
        text = format_summary(self.state(sim=True), a)
        self.assertIn('SIMULATED DATA', text)
        self.assertIn('STALE', text)
        self.assertIn('M7', text)


if __name__ == '__main__':
    unittest.main()
