"""Keyboard layout for manual control (pure Python).

Directions are in the OPERATOR's sense. Which way a motor physically turns for "+" is set
by the uncalibrated per-channel polarity in arm_bringup/config/channels.yaml.

Layout: one letter pair per channel, upper row = "+", lower row = "-".

    M1 q/a   M2 w/s   M3 e/d   M4 r/f   M5 t/g   M6 y/h   M7 u/j
"""
from typing import Dict, Tuple

# key -> (channel number 1..7, operator direction -1/+1)
MOVE_KEYS: Dict[str, Tuple[int, int]] = {
    'q': (1, +1), 'a': (1, -1),
    'w': (2, +1), 's': (2, -1),
    'e': (3, +1), 'd': (3, -1),
    'r': (4, +1), 'f': (4, -1),
    't': (5, +1), 'g': (5, -1),
    'y': (6, +1), 'h': (6, -1),
    'u': (7, +1), 'j': (7, -1),
}

KEY_STOP = ' '            # normal stop (space bar)
KEY_ESTOP = 'x'           # emergency-stop REQUEST (software; not a physical E-stop)
KEY_REARM = 'Z'           # re-arm after ESTOP (capital Z, deliberately harder to hit)
KEYS_LEVEL_UP = ('+', '=')
KEYS_LEVEL_DOWN = ('-', '_')
KEY_HELP = '?'
KEY_QUIT = 'Q'            # capital Q; Ctrl-C also quits


def help_text(hold_window_s: float) -> str:
    return f'''
=== Team Vyadh arm keyboard teleop (manual control) ===
 Move (hold the key; letters = channel, top row "+" / bottom row "-" in OPERATOR sense):
   M1 turntable      q / a        M2 shoulder   w / s       M3 elbow    e / d
   M4 wrist pitch    r / f        M5 wrist A    t / g       M6 wrist B  y / h
   M7 gripper        u / j
 SPACE   normal stop (all channels)
 x       EMERGENCY-STOP REQUEST (software only - NOT a physical E-stop; latches until re-arm)
 Z       re-arm after emergency stop (never starts motion by itself)
 + / -   speed level up / down      ?  this help      Q  quit (Ctrl-C also quits)
 Notes:
  * A terminal cannot report key RELEASE. Each key press/auto-repeat keeps that channel
    moving for {hold_window_s:.2f} s; when the key is released the channel stops requesting
    movement after about that long (plus the firmware's own ramp-down). This timer is NOT
    the hardware watchdog - the STM32 command timeout is.
  * Direction signs/polarity are UNCALIBRATED until verified on hardware.
'''
