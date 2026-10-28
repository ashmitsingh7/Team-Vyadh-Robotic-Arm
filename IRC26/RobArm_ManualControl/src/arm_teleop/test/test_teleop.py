"""Teleop logic, key map and terminal input (uses a real pseudo-terminal where available)."""
import os
import sys
import termios
import unittest

from arm_teleop import key_map
from arm_teleop.teleop_core import (ACTION_ESTOP, ACTION_MOVE, ACTION_REARM, ACTION_STOP, Intent,
                                    TeleopCore)
from arm_teleop.terminal_input import TerminalError, TerminalInput, strip_escape_sequences


class TestKeyMap(unittest.TestCase):
    def test_all_seven_channels_both_directions_unique(self):
        self.assertEqual(sorted(set(MOVE for MOVE in key_map.MOVE_KEYS.values())),
                         sorted((c, d) for c in range(1, 8) for d in (-1, 1)))
        self.assertEqual(len(key_map.MOVE_KEYS), 14)

    def test_special_keys_do_not_collide_with_move_keys(self):
        specials = {key_map.KEY_STOP, key_map.KEY_ESTOP, key_map.KEY_REARM, key_map.KEY_HELP,
                    key_map.KEY_QUIT, *key_map.KEYS_LEVEL_UP, *key_map.KEYS_LEVEL_DOWN}
        self.assertFalse(specials & set(key_map.MOVE_KEYS))
        self.assertEqual(len(specials), 4 + 1 + len(key_map.KEYS_LEVEL_UP) + len(key_map.KEYS_LEVEL_DOWN))

    def test_estop_and_rearm_are_separate_keys_and_rearm_is_hard_to_hit(self):
        self.assertNotEqual(key_map.KEY_ESTOP, key_map.KEY_REARM)
        self.assertTrue(key_map.KEY_REARM.isupper())

    def test_help_documents_latency_and_not_a_watchdog(self):
        text = key_map.help_text(0.6)
        self.assertIn('0.60', text)
        self.assertIn('NOT a physical E-stop', text)
        self.assertIn('NOT\n    the hardware watchdog', text)

    def test_core_constants_match_msg_file(self):
        import re
        from pathlib import Path
        msg = (Path(__file__).resolve().parents[2] / 'arm_interfaces' / 'msg' / 'UserInput.msg').read_text()
        consts = dict((k, int(v)) for k, v in re.findall(r'uint8 (ACTION_\w+)=(\d+)', msg))
        self.assertEqual((consts['ACTION_MOVE'], consts['ACTION_STOP'], consts['ACTION_ESTOP'],
                          consts['ACTION_REARM']), (ACTION_MOVE, ACTION_STOP, ACTION_ESTOP, ACTION_REARM))


class TestTeleopCore(unittest.TestCase):
    def test_every_move_key_emits_correct_channel_and_direction(self):
        for key, (ch, d) in key_map.MOVE_KEYS.items():
            core = TeleopCore()
            self.assertEqual(core.on_key(key, 0.0), [Intent(ACTION_MOVE, ch, d, 0.5)], key)

    def test_hold_refreshes_then_releases_exactly_once(self):
        core = TeleopCore(hold_window_s=0.6)
        core.on_key('q', 0.0)
        self.assertEqual(core.tick(0.30), [Intent(ACTION_MOVE, 1, 1, 0.5)])
        core.on_key('q', 0.40)                      # auto-repeat extends the window
        self.assertEqual(core.tick(0.90), [Intent(ACTION_MOVE, 1, 1, 0.5)])
        self.assertEqual(core.tick(1.00), [Intent(ACTION_MOVE, 1, 0, 0.0)])   # lapsed: release
        self.assertEqual(core.tick(1.05), [])                                  # only once

    def test_stop_latency_equals_hold_window(self):
        core = TeleopCore(hold_window_s=0.6)
        core.on_key('w', 0.0)
        step = 0.05
        t = 0.0
        released_at = None
        while t < 2.0:
            t = round(t + step, 6)
            if any(i.direction == 0 for i in core.tick(t)):
                released_at = t
                break
        self.assertAlmostEqual(released_at, 0.6, delta=step + 1e-9)

    def test_independent_channels_release_independently(self):
        core = TeleopCore(hold_window_s=0.6)
        core.on_key('t', 0.0)
        core.on_key('y', 0.4)
        out = core.tick(0.65)
        self.assertIn(Intent(ACTION_MOVE, 5, 0, 0.0), out)     # M5 released
        self.assertIn(Intent(ACTION_MOVE, 6, 1, 0.5), out)     # M6 still held

    def test_direction_reversal_on_same_channel(self):
        core = TeleopCore()
        core.on_key('q', 0.0)
        core.on_key('a', 0.1)
        self.assertEqual(core.held_channels, {1: -1})

    def test_stop_clears_everything(self):
        core = TeleopCore()
        core.on_key('q', 0.0)
        core.on_key('u', 0.0)
        self.assertEqual(core.on_key(' ', 0.1), [Intent(ACTION_STOP)])
        self.assertEqual(core.tick(0.11), [])

    def test_estop_and_rearm_are_distinct_actions(self):
        core = TeleopCore()
        self.assertEqual(core.on_key('x', 0.0), [Intent(ACTION_ESTOP)])
        self.assertEqual(core.on_key('Z', 0.1), [Intent(ACTION_REARM)])
        self.assertNotEqual(ACTION_STOP, ACTION_ESTOP)

    def test_rearm_does_not_resume_held_motion(self):
        core = TeleopCore()
        core.on_key('q', 0.0)
        core.on_key('x', 0.1)
        core.on_key('Z', 0.2)
        self.assertEqual(core.tick(0.21), [])

    def test_level_changes_are_bounded_and_apply_to_held(self):
        core = TeleopCore(level=0.5)
        for _ in range(20):
            core.on_key('+', 0.0)
        self.assertEqual(core.level, 1.0)
        for _ in range(20):
            core.on_key('-', 0.0)
        self.assertEqual(core.level, 0.1)
        core.on_key('q', 0.0)
        core.on_key('+', 0.0)
        self.assertEqual(core.tick(0.1)[0].level, 0.2)

    def test_unknown_keys_ignored_help_and_quit(self):
        core = TeleopCore()
        for k in ('z', '1', '\n', '\x03', 'é'):
            self.assertEqual(core.on_key(k, 0.0), [])
        core.on_key('?', 0.0)
        self.assertTrue(core.drain_messages())
        self.assertFalse(core.quit_requested)
        core.on_key('Q', 0.0)
        self.assertTrue(core.quit_requested)

    def test_shutdown_publishes_stop_and_clears(self):
        core = TeleopCore()
        core.on_key('q', 0.0)
        self.assertEqual(core.shutdown_intents(), [Intent(ACTION_STOP)])
        self.assertEqual(core.tick(0.1), [])

    def test_bad_construction(self):
        for kw in (dict(hold_window_s=0), dict(level=0.0), dict(level=1.5), dict(level_step=0)):
            with self.assertRaises(ValueError):
                TeleopCore(**kw)


class TestEscapeStripping(unittest.TestCase):
    def test_arrow_keys_and_lone_escape_removed(self):
        self.assertEqual(strip_escape_sequences('\x1b[A\x1b[1;5Cq'), 'q')
        self.assertEqual(strip_escape_sequences('a\x1bOPb'), 'ab')
        self.assertEqual(strip_escape_sequences('\x1b'), '')
        self.assertEqual(strip_escape_sequences('plain'), 'plain')


@unittest.skipUnless(hasattr(os, 'openpty'), 'needs a POSIX pseudo-terminal')
class TestTerminalInputPty(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = os.openpty()

    def tearDown(self):
        os.close(self.master)
        os.close(self.slave)

    def test_reads_keys_and_restores_settings(self):
        before = termios.tcgetattr(self.slave)
        with TerminalInput(fd=self.slave) as term:
            self.assertNotEqual(termios.tcgetattr(self.slave), before)    # cbreak active
            self.assertEqual(term.read_keys(), [])                         # non-blocking when idle
            os.write(self.master, b'qa \x1b[Ax')
            self.assertEqual(term.read_keys(), ['q', 'a', ' ', 'x'])
        self.assertEqual(termios.tcgetattr(self.slave), before)           # restored

    def test_restored_after_exception(self):
        before = termios.tcgetattr(self.slave)
        with self.assertRaises(RuntimeError):
            with TerminalInput(fd=self.slave):
                raise RuntimeError('boom')
        self.assertEqual(termios.tcgetattr(self.slave), before)

    def test_restore_is_idempotent(self):
        term = TerminalInput(fd=self.slave)
        term.restore()
        with term:
            pass
        term.restore()

    def test_non_tty_rejected_with_helpful_message(self):
        r, w = os.pipe()
        try:
            with self.assertRaises(TerminalError) as cm:
                TerminalInput(fd=r).__enter__()
            self.assertIn('terminal', str(cm.exception))
        finally:
            os.close(r)
            os.close(w)


if __name__ == '__main__':
    unittest.main()
