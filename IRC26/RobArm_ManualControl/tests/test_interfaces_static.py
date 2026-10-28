"""Static (no-ROS) tests for arm_interfaces and the channel table.

Run from the repo root:  python3 -m unittest discover -s tests -v
These do NOT replace `colcon build` / `colcon test` on a ROS 2 Jazzy machine.
"""
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PKG = REPO / 'src' / 'arm_interfaces'
sys.path.insert(0, str(REPO / 'tests'))
sys.path.insert(0, str(REPO / 'src' / 'arm_control'))

import check_interfaces as ci  # noqa: E402
from arm_control import channels  # noqa: E402


class TestRealInterfaces(unittest.TestCase):
    def test_no_problems_in_repo_interfaces(self):
        self.assertEqual(ci.run_checks(PKG, channels.CHANNEL_COUNT), [])

    def test_expected_message_set(self):
        names = {p.stem for p in (PKG / 'msg').glob('*.msg')}
        self.assertEqual(names, {'UserInput', 'MotorCommand', 'ArmCommand',
                                 'MotorState', 'ArmState', 'ArmDiagnostics'})

    def test_arm_command_is_full_snapshot_of_seven(self):
        text = (PKG / 'msg' / 'ArmCommand.msg').read_text()
        self.assertIn('MotorCommand[<=7] motors', text)
        self.assertIn('uint8 NUM_CHANNELS=7', text)
        self.assertIn('FULL SEVEN-CHANNEL SNAPSHOT', text)

    def test_no_free_form_strings(self):
        for p in (PKG / 'msg').glob('*.msg'):
            code = '\n'.join(l.split('#', 1)[0] for l in p.read_text().splitlines())
            self.assertIsNone(re.search(r'^\s*w?string\s', code, re.M), p.name)

    def test_channel_constants_match_channel_table(self):
        msg = ci.parse_msg(PKG / 'msg' / 'UserInput.msg', [])
        for ch in channels.CHANNELS:
            self.assertEqual(msg.constants[f'CHANNEL_{ch.name}'][1], ch.number)


class TestChannelTable(unittest.TestCase):
    def test_seven_unique_channels_in_order(self):
        self.assertEqual(channels.CHANNEL_COUNT, 7)
        self.assertEqual(channels.CHANNEL_NAMES, ('M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7'))
        self.assertEqual(channels.CHANNEL_NUMBERS, (1, 2, 3, 4, 5, 6, 7))

    def test_feedback_kinds_match_spec(self):
        fb = {c.name: c.feedback for c in channels.CHANNELS}
        self.assertEqual(fb['M1'], 'encoder')
        self.assertEqual(fb['M2'], 'angle')
        self.assertEqual(fb['M3'], 'angle')
        self.assertEqual([fb[m] for m in ('M4', 'M5', 'M6')], ['encoder'] * 3)
        self.assertEqual(fb['M7'], 'current')

    def test_index_round_trip(self):
        for n in range(1, 8):
            self.assertEqual(channels.index_to_number(channels.number_to_index(n)), n)

    def test_bad_numbers_rejected(self):
        for bad in (0, 8, -1, 1.0, '1', None, True):
            with self.assertRaises(ValueError, msg=repr(bad)):
                channels.number_to_index(bad)
        for bad in (-1, 7, 0.0, None, False):
            with self.assertRaises(ValueError, msg=repr(bad)):
                channels.index_to_number(bad)

    def test_name_lookup(self):
        self.assertEqual(channels.name_to_number('m7'), 7)
        for bad in ('M0', 'M8', '', 'wrist'):
            with self.assertRaises(ValueError):
                channels.name_to_number(bad)


class TestCheckerCatchesMistakes(unittest.TestCase):
    """Negative tests: the checker must flag deliberately broken copies of the package."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.pkg = self.tmp / 'arm_interfaces'
        shutil.copytree(PKG, self.pkg)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def problems(self):
        return ci.run_checks(self.pkg, channels.CHANNEL_COUNT)

    def edit(self, msg, old, new):
        f = self.pkg / 'msg' / f'{msg}.msg'
        text = f.read_text()
        self.assertIn(old, text)
        f.write_text(text.replace(old, new, 1))

    def test_baseline_clean(self):
        self.assertEqual(self.problems(), [])

    def test_unbounded_channel_array(self):
        self.edit('ArmCommand', 'MotorCommand[<=7]', 'MotorCommand[]')
        self.assertTrue(any('bounded' in p for p in self.problems()))

    def test_wrong_bound(self):
        self.edit('ArmState', 'MotorState[<=7]', 'MotorState[<=8]')
        self.assertTrue(any('bounded' in p for p in self.problems()))

    def test_num_channels_mismatch(self):
        self.edit('ArmCommand', 'NUM_CHANNELS=7', 'NUM_CHANNELS=6')
        self.assertTrue(any('NUM_CHANNELS' in p for p in self.problems()))

    def test_invalid_not_zero(self):
        self.edit('ArmCommand', 'COMMAND_TYPE_INVALID=0', 'COMMAND_TYPE_INVALID=5')
        self.assertTrue(any('COMMAND_TYPE_INVALID' in p for p in self.problems()))

    def test_flag_not_power_of_two(self):
        self.edit('MotorState', 'VALID_CURRENT=8', 'VALID_CURRENT=6')
        self.assertTrue(any('power of two' in p for p in self.problems()))

    def test_duplicate_flag_bit(self):
        self.edit('ArmState', 'FAULT_DRIVER=32', 'FAULT_DRIVER=16')
        self.assertTrue(any('duplicate bit' in p for p in self.problems()))

    def test_free_form_string_rejected(self):
        (self.pkg / 'msg' / 'ArmCommand.msg').write_text(
            (self.pkg / 'msg' / 'ArmCommand.msg').read_text() + '\nstring note\n')
        self.assertTrue(any('free-form string' in p for p in self.problems()))

    def test_constant_out_of_range(self):
        self.edit('UserInput', 'ACTION_STOP=2', 'ACTION_STOP=300')
        self.assertTrue(any('outside uint8' in p for p in self.problems()))

    def test_message_missing_from_cmake(self):
        f = self.pkg / 'CMakeLists.txt'
        f.write_text(f.read_text().replace('  "msg/ArmDiagnostics.msg"\n', ''))
        self.assertTrue(any('CMakeLists.txt' in p for p in self.problems()))

    def test_unknown_type(self):
        self.edit('ArmState', 'MotorState[<=7]', 'MotorStatus[<=7]')
        self.assertTrue(any('unknown type' in p for p in self.problems()))


if __name__ == '__main__':
    unittest.main()
