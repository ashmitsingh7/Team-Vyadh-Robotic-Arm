"""Teleop logic: keys -> operator intents with a hold window (pure Python, no ROS, no terminal).

A terminal delivers key presses and auto-repeats but never key release. This core therefore
treats each press/repeat as "keep moving for hold_window_s" and emits an explicit RELEASE
when the window lapses without a refresh. The hold window is NOT a safety watchdog: the
manual-control input timeout and, finally, the STM32 command timeout are the protections.

The input source is replaceable: anything that yields key characters (or, later, true
press/release events) can drive this class.
"""
from dataclasses import dataclass
from typing import Dict, List, Tuple

from arm_teleop.key_map import (KEY_ESTOP, KEY_HELP, KEY_QUIT, KEY_REARM, KEY_STOP,
                                KEYS_LEVEL_DOWN, KEYS_LEVEL_UP, MOVE_KEYS, help_text)

# Mirrors UserInput.ACTION_* (tests cross-check against the .msg file).
ACTION_MOVE = 1
ACTION_STOP = 2
ACTION_ESTOP = 3
ACTION_REARM = 4


@dataclass(frozen=True)
class Intent:
    action: int
    channel: int = 0
    direction: int = 0
    level: float = 0.0


class TeleopCore:
    def __init__(self, hold_window_s: float = 0.6, level: float = 0.5, level_step: float = 0.1,
                 min_level: float = 0.1, max_level: float = 1.0):
        if hold_window_s <= 0:
            raise ValueError('hold_window_s must be positive')
        if not 0.0 < min_level <= level <= max_level <= 1.0 or level_step <= 0:
            raise ValueError('need 0 < min_level <= level <= max_level <= 1 and level_step > 0')
        self.hold_window_s = hold_window_s
        self.level = level
        self.level_step = level_step
        self.min_level = min_level
        self.max_level = max_level
        self.quit_requested = False
        self._held: Dict[int, Tuple[int, float]] = {}   # channel -> (direction, expires_at)
        self.messages: List[str] = []                    # operator-facing text, drained by the node

    def drain_messages(self) -> List[str]:
        msgs, self.messages = self.messages, []
        return msgs

    @property
    def held_channels(self) -> Dict[int, int]:
        return {ch: d for ch, (d, _) in self._held.items()}

    def on_key(self, key: str, now: float) -> List[Intent]:
        """Handle one key character; returns intents to publish immediately."""
        if key in MOVE_KEYS:
            ch, direction = MOVE_KEYS[key]
            self._held[ch] = (direction, now + self.hold_window_s)
            return [Intent(ACTION_MOVE, ch, direction, self.level)]
        if key == KEY_STOP:
            self._held.clear()
            self.messages.append('STOP')
            return [Intent(ACTION_STOP)]
        if key == KEY_ESTOP:
            self._held.clear()
            self.messages.append('EMERGENCY-STOP REQUESTED (software). Press Z to re-arm.')
            return [Intent(ACTION_ESTOP)]
        if key == KEY_REARM:
            self._held.clear()
            self.messages.append('REARM requested')
            return [Intent(ACTION_REARM)]
        if key in KEYS_LEVEL_UP:
            self.level = min(self.max_level, round(self.level + self.level_step, 6))
            self.messages.append(f'speed level {self.level:.2f}')
            return []
        if key in KEYS_LEVEL_DOWN:
            self.level = max(self.min_level, round(self.level - self.level_step, 6))
            self.messages.append(f'speed level {self.level:.2f}')
            return []
        if key == KEY_HELP:
            self.messages.append(help_text(self.hold_window_s))
            return []
        if key == KEY_QUIT:
            self.quit_requested = True
            return []
        return []   # unknown keys are ignored

    def tick(self, now: float) -> List[Intent]:
        """Periodic refresh: re-send held moves; send one RELEASE when a hold window lapses."""
        out: List[Intent] = []
        for ch in sorted(self._held):
            direction, expires = self._held[ch]
            if now >= expires:
                del self._held[ch]
                out.append(Intent(ACTION_MOVE, ch, 0, 0.0))     # explicit release
            else:
                out.append(Intent(ACTION_MOVE, ch, direction, self.level))
        return out

    def shutdown_intents(self) -> List[Intent]:
        """What to publish on exit: a normal stop."""
        self._held.clear()
        return [Intent(ACTION_STOP)]
