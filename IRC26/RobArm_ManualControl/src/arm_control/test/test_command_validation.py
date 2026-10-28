"""Malformed or missing channel data must never produce motion (spec section 7)."""
import copy
import math
import unittest
from types import SimpleNamespace as NS

from arm_control.command_validation import (validate_arm_command, validate_arm_state,
                                            validate_motor_command, validate_user_input)


def motors(overrides=None):
    ms = [NS(channel=c, direction=0, demand=0.0) for c in range(1, 8)]
    for c, (d, dem) in (overrides or {}).items():
        ms[c - 1] = NS(channel=c, direction=d, demand=dem)
    return ms


def cmd(ctype=1, seq=0, ms=None):
    return NS(command_type=ctype, sequence=seq, motors=motors() if ms is None else ms)


def move(channel=1, direction=1, level=0.5, action=1):
    return NS(action=action, channel=channel, direction=direction, level=level)


class TestArmCommand(unittest.TestCase):
    def assertRejected(self, c, fragment=''):
        problems = validate_arm_command(c)
        self.assertTrue(problems, f'expected rejection of {c}')
        if fragment:
            self.assertTrue(any(fragment in p for p in problems), problems)

    def test_valid_snapshots(self):
        self.assertEqual(validate_arm_command(cmd()), [])
        self.assertEqual(validate_arm_command(cmd(ms=motors({3: (-1, 0.2), 7: (1, 1.0)}))), [])

    def test_any_order_is_fine(self):
        ms = list(reversed(motors({2: (1, 0.3)})))
        self.assertEqual(validate_arm_command(cmd(ms=ms)), [])

    def test_missing_channel(self):
        ms = motors({1: (1, 0.5)})
        del ms[3]
        self.assertRejected(cmd(ms=ms), 'exactly 7')

    def test_missing_channel_replaced_by_duplicate(self):
        ms = motors({1: (1, 0.5)})
        ms[3] = NS(channel=1, direction=1, demand=0.5)
        self.assertRejected(cmd(ms=ms), 'duplicate')

    def test_empty_and_extra_entries(self):
        self.assertRejected(cmd(ms=[]), 'exactly 7')
        self.assertRejected(cmd(ms=motors() + [NS(channel=1, direction=0, demand=0.0)]), 'exactly 7')

    def test_six_entries_eight_entries(self):
        self.assertRejected(cmd(ms=motors()[:6]))
        self.assertRejected(cmd(ms=motors() + [NS(channel=8, direction=0, demand=0.0)]))

    def test_bad_channel_numbers(self):
        for bad in (0, 8, -1, 255, 1.0, None, True):
            ms = motors()
            ms[0] = NS(channel=bad, direction=0, demand=0.0)
            self.assertRejected(cmd(ms=ms))

    def test_invalid_command_type(self):
        for bad in (0, 5, -1, 255, None, 1.0, True):
            self.assertRejected(cmd(ctype=bad))

    def test_bad_sequence(self):
        for bad in (-1, 2 ** 32, 1.5, None, True):
            self.assertRejected(cmd(seq=bad))

    def test_nan_inf_demand(self):
        for bad in (math.nan, math.inf, -math.inf):
            self.assertRejected(cmd(ms=motors({2: (1, bad)})))

    def test_demand_out_of_range(self):
        for bad in (1.0001, -0.1, 2.0, '0.5', None):
            self.assertRejected(cmd(ms=motors({2: (1, bad)})))

    def test_bad_direction(self):
        for bad in (2, -2, 0.5, None, True, '1'):
            self.assertRejected(cmd(ms=motors({2: (bad, 0.5)})))

    def test_direction_demand_consistency(self):
        self.assertRejected(cmd(ms=motors({2: (0, 0.5)})), 'inconsistent')
        self.assertRejected(cmd(ms=motors({2: (1, 0.0)})), 'inconsistent')
        self.assertRejected(cmd(ms=motors({2: (-1, 0.0)})), 'inconsistent')

    def test_missing_attributes_reported_not_raised(self):
        self.assertRejected(NS())
        self.assertRejected(NS(command_type=1, sequence=0))
        self.assertRejected(cmd(ms=[NS(channel=1)] * 7))

    def test_motors_not_a_sequence(self):
        self.assertRejected(cmd(ms=5))

    def test_stop_and_estop_honoured_even_with_garbage_motors(self):
        # A stop must never be blockable by malformed channel data; motors are never sent.
        for ctype in (2, 3):
            for garbage in ([], motors({1: (1, 0.9)}), 'junk', [NS(channel=99)]):
                self.assertEqual(validate_arm_command(cmd(ctype=ctype, ms=garbage)), [], (ctype, garbage))

    def test_rearm_requires_valid_zero_snapshot(self):
        self.assertEqual(validate_arm_command(cmd(ctype=4)), [])
        self.assertRejected(cmd(ctype=4, ms=motors({1: (1, 0.2)})), 'all-zero')
        self.assertRejected(cmd(ctype=4, ms=[]))

    def test_validation_does_not_mutate(self):
        c = cmd(ms=motors({1: (1, 0.2)}))
        before = copy.deepcopy(c)
        validate_arm_command(c)
        self.assertEqual(c, before)


class TestMotorCommand(unittest.TestCase):
    def test_zero_default_message_is_invalid(self):
        # Default MotorCommand has channel 0: must be rejected.
        self.assertTrue(validate_motor_command(NS(channel=0, direction=0, demand=0.0)))


class TestUserInput(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(validate_user_input(move()), [])
        self.assertEqual(validate_user_input(move(direction=0, level=0.0)), [])   # release

    def test_default_message_invalid(self):
        self.assertTrue(validate_user_input(NS(action=0, channel=0, direction=0, level=0.0)))

    def test_bad_actions(self):
        for bad in (0, 5, 255, -1, None, 1.0, True):
            self.assertTrue(validate_user_input(move(action=bad)), bad)

    def test_move_bad_fields(self):
        for kw in (dict(channel=0), dict(channel=8), dict(direction=2), dict(direction=-2),
                   dict(level=math.nan), dict(level=math.inf), dict(level=1.1), dict(level=-0.1),
                   dict(level=0.0), dict(direction=0, level=0.5)):
            self.assertTrue(validate_user_input(move(**kw)), kw)

    def test_stop_estop_rearm_ignore_other_fields(self):
        # Garbage in other fields must never prevent a stop/estop.
        for action in (2, 3, 4):
            self.assertEqual(validate_user_input(NS(action=action, channel=99, direction=7,
                                                    level=math.nan)), [])


class TestArmState(unittest.TestCase):
    def state(self, channels=range(1, 8), fw=2):
        return NS(firmware_state=fw, motors=[NS(channel=c) for c in channels])

    def test_valid(self):
        self.assertEqual(validate_arm_state(self.state()), [])

    def test_missing_duplicate_extra(self):
        self.assertTrue(validate_arm_state(self.state(range(1, 7))))
        self.assertTrue(validate_arm_state(self.state([1, 1, 2, 3, 4, 5, 6])))
        self.assertTrue(validate_arm_state(self.state(list(range(1, 8)) + [1])))
        self.assertTrue(validate_arm_state(self.state([0, 1, 2, 3, 4, 5, 6])))

    def test_bad_firmware_state(self):
        self.assertTrue(validate_arm_state(self.state(fw=7)))


if __name__ == '__main__':
    unittest.main()
