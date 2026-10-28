"""Channel table for the seven arm motor channels (pure Python, no ROS imports).

Single place in ROS-side code that fixes the M1..M7 naming and numbering.
Message fields use the human channel number 1..7 (M1 = 1). Python arrays that
mirror a seven-channel message use index = channel_number - 1.

The ``feedback`` entries describe which sensor the hardware *has*; whether a
reading is valid at runtime is reported separately by MotorState.valid_flags.
Nothing here encodes polarity, limits or calibration - those live in
arm_bringup/config/channels.yaml and are UNCALIBRATED until hardware tests.
"""
from dataclasses import dataclass
from typing import Tuple

CHANNEL_COUNT = 7

FEEDBACK_ENCODER = 'encoder'   # motor-side incremental encoder counts (not a joint angle)
FEEDBACK_ANGLE = 'angle'       # AS5600 joint angle at the pivot
FEEDBACK_CURRENT = 'current'   # current sensor only; no position


@dataclass(frozen=True)
class Channel:
    number: int          # 1..7, as used in messages
    name: str            # 'M1'..'M7'
    mechanism: str
    feedback: str


CHANNELS: Tuple[Channel, ...] = (
    Channel(1, 'M1', 'Turntable rotation about base Z', FEEDBACK_ENCODER),
    Channel(2, 'M2', 'Shoulder linear actuator', FEEDBACK_ANGLE),
    Channel(3, 'M3', 'Elbow linear actuator', FEEDBACK_ANGLE),
    Channel(4, 'M4', 'Wrist pitch motor', FEEDBACK_ENCODER),
    # M5/M6 stay independent: no roll/yaw mapping until the differential geometry is given.
    Channel(5, 'M5', 'Wrist differential motor A', FEEDBACK_ENCODER),
    Channel(6, 'M6', 'Wrist differential motor B', FEEDBACK_ENCODER),
    Channel(7, 'M7', 'Gripper lead-screw motor', FEEDBACK_CURRENT),
)

CHANNEL_NUMBERS = tuple(c.number for c in CHANNELS)
CHANNEL_NAMES = tuple(c.name for c in CHANNELS)


def number_to_index(number: int) -> int:
    """Channel number 1..7 -> array index 0..6. Raises ValueError if out of range."""
    if isinstance(number, bool) or not isinstance(number, int) or not 1 <= number <= CHANNEL_COUNT:
        raise ValueError(f'channel number must be an int in 1..{CHANNEL_COUNT}, got {number!r}')
    return number - 1


def index_to_number(index: int) -> int:
    """Array index 0..6 -> channel number 1..7. Raises ValueError if out of range."""
    if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < CHANNEL_COUNT:
        raise ValueError(f'channel index must be an int in 0..{CHANNEL_COUNT - 1}, got {index!r}')
    return index + 1


def name_to_number(name: str) -> int:
    """'M1'..'M7' (case-insensitive) -> 1..7. Raises ValueError for anything else."""
    try:
        return CHANNEL_NAMES.index(str(name).upper()) + 1
    except ValueError:
        raise ValueError(f'unknown channel name {name!r}; expected one of {CHANNEL_NAMES}') from None
