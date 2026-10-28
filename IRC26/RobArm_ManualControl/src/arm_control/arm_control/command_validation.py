"""Validation of UserInput / ArmCommand / ArmState structures (pure Python, no ROS imports).

All functions are duck-typed: they accept ROS message objects or any object with the same
attributes, and return a list of human-readable problems (empty list = valid). They never
raise for malformed input; a missing attribute is reported as a problem.

Policy (see docs/ros_interface_spec.md section 2):
  * ArmCommand is a full seven-channel snapshot; any violation rejects the WHOLE command.
  * STOP and ESTOP are honoured on `command_type` + `sequence` alone. Their `motors` content
    is never transmitted, so malformed channel data cannot block a stop and cannot cause
    motion. REARM is stricter: it needs a valid all-zero snapshot.
  * UserInput STOP / ESTOP / REARM are honoured on `action` alone (other fields ignored).
"""
import math
from typing import List

from arm_control.channels import CHANNEL_COUNT

COMMAND_TYPE_INVALID = 0
COMMAND_TYPE_SETPOINT = 1
COMMAND_TYPE_STOP = 2
COMMAND_TYPE_ESTOP = 3
COMMAND_TYPE_REARM = 4

ACTION_INVALID = 0
ACTION_MOVE = 1
ACTION_STOP = 2
ACTION_ESTOP = 3
ACTION_REARM = 4

FW_STATE_MAX = 6
_UINT32_MAX = 2 ** 32 - 1


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _get(obj, name, problems, label):
    try:
        return getattr(obj, name)
    except AttributeError:
        problems.append(f'{label}: missing attribute {name!r}')
        return None


def validate_motor_command(motor, label: str = 'motor') -> List[str]:
    problems: List[str] = []
    channel = _get(motor, 'channel', problems, label)
    direction = _get(motor, 'direction', problems, label)
    demand = _get(motor, 'demand', problems, label)
    if problems:
        return problems
    if not _is_int(channel) or not 1 <= channel <= CHANNEL_COUNT:
        problems.append(f'{label}: channel {channel!r} not in 1..{CHANNEL_COUNT}')
    if not _is_int(direction) or direction not in (-1, 0, 1):
        problems.append(f'{label}: direction {direction!r} not in (-1, 0, 1)')
    if not _is_finite_number(demand) or not 0.0 <= demand <= 1.0:
        problems.append(f'{label}: demand {demand!r} not a finite number in [0, 1]')
    if not problems:
        if (demand == 0.0) != (direction == 0):
            problems.append(f'{label}: demand {demand} inconsistent with direction {direction} '
                            '(demand == 0 iff direction == 0)')
    return problems


def validate_arm_command(cmd) -> List[str]:
    """Return problems with an ArmCommand-like object; [] means safe to transmit."""
    problems: List[str] = []
    ctype = _get(cmd, 'command_type', problems, 'ArmCommand')
    seq = _get(cmd, 'sequence', problems, 'ArmCommand')
    if problems:
        return problems
    if not _is_int(ctype) or not COMMAND_TYPE_SETPOINT <= ctype <= COMMAND_TYPE_REARM:
        problems.append(f'command_type {ctype!r} is not SETPOINT/STOP/ESTOP/REARM')
    if not _is_int(seq) or not 0 <= seq <= _UINT32_MAX:
        problems.append(f'sequence {seq!r} is not a uint32')
    if problems:
        return problems
    if ctype in (COMMAND_TYPE_STOP, COMMAND_TYPE_ESTOP):
        return []   # motors content is ignored and never transmitted

    motors = _get(cmd, 'motors', problems, 'ArmCommand')
    if motors is None:
        return problems
    try:
        motors = list(motors)
    except TypeError:
        return problems + ['motors is not a sequence']
    if len(motors) != CHANNEL_COUNT:
        problems.append(f'motors has {len(motors)} entries, exactly {CHANNEL_COUNT} required')
    seen = []
    for i, motor in enumerate(motors):
        sub = validate_motor_command(motor, f'motors[{i}]')
        problems.extend(sub)
        if not sub:
            seen.append(motor.channel)
    if len(set(seen)) != len(seen):
        problems.append(f'duplicate channels in motors: {sorted(seen)}')
    if not problems and sorted(seen) != list(range(1, CHANNEL_COUNT + 1)):
        problems.append(f'channels {sorted(seen)} do not cover 1..{CHANNEL_COUNT}')
    if not problems and ctype == COMMAND_TYPE_REARM:
        if any(m.direction != 0 or m.demand != 0.0 for m in motors):
            problems.append('REARM requires an all-zero motor snapshot')
    return problems


def validate_user_input(ui) -> List[str]:
    """Return problems with a UserInput-like object."""
    problems: List[str] = []
    action = _get(ui, 'action', problems, 'UserInput')
    if problems:
        return problems
    if not _is_int(action) or not ACTION_MOVE <= action <= ACTION_REARM:
        return [f'action {action!r} is not MOVE/STOP/ESTOP/REARM']
    if action != ACTION_MOVE:
        return []   # STOP / ESTOP / REARM are honoured on `action` alone
    channel = _get(ui, 'channel', problems, 'UserInput')
    direction = _get(ui, 'direction', problems, 'UserInput')
    level = _get(ui, 'level', problems, 'UserInput')
    if problems:
        return problems
    if not _is_int(channel) or not 1 <= channel <= CHANNEL_COUNT:
        problems.append(f'MOVE channel {channel!r} not in 1..{CHANNEL_COUNT}')
    if not _is_int(direction) or direction not in (-1, 0, 1):
        problems.append(f'MOVE direction {direction!r} not in (-1, 0, 1)')
    if not _is_finite_number(level) or not 0.0 <= level <= 1.0:
        problems.append(f'MOVE level {level!r} not a finite number in [0, 1]')
    if not problems:
        if direction != 0 and level <= 0.0:
            problems.append('MOVE with non-zero direction needs level > 0')
        if direction == 0 and level != 0.0:
            problems.append('MOVE release (direction 0) needs level == 0')
    return problems


def validate_arm_state(state) -> List[str]:
    """Check an ArmState-like object has exactly seven distinct, in-range channel entries."""
    problems: List[str] = []
    fw = _get(state, 'firmware_state', problems, 'ArmState')
    motors = _get(state, 'motors', problems, 'ArmState')
    if problems:
        return problems
    if not _is_int(fw) or not 0 <= fw <= FW_STATE_MAX:
        problems.append(f'firmware_state {fw!r} out of range')
    try:
        motors = list(motors)
    except TypeError:
        return problems + ['motors is not a sequence']
    channels = []
    for i, m in enumerate(motors):
        ch = getattr(m, 'channel', None)
        if not _is_int(ch) or not 1 <= ch <= CHANNEL_COUNT:
            problems.append(f'motors[{i}]: channel {ch!r} not in 1..{CHANNEL_COUNT}')
        else:
            channels.append(ch)
    if sorted(channels) != list(range(1, CHANNEL_COUNT + 1)):
        problems.append(f'motor channels {sorted(channels)} do not cover 1..{CHANNEL_COUNT} exactly once')
    return problems
