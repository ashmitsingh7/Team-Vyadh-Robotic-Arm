"""Map operator intent for one channel to a firmware-convention target (pure Python).

Applies the per-channel polarity and maximum demand from configuration. Both are
UNCALIBRATED placeholders: the mapper never assumes motors share polarity or that
"positive" is mechanically correct.
"""
from dataclasses import dataclass

from arm_control.control_config import ChannelConfig


class MappingError(ValueError):
    """Intent cannot be mapped (e.g. channel disabled in configuration)."""


@dataclass(frozen=True)
class ChannelTarget:
    direction: int    # firmware convention: -1, 0, +1
    demand: float     # fraction of firmware full scale, [0, 1]


ZERO_TARGET = ChannelTarget(0, 0.0)


def map_move(channel: int, direction: int, level: float, cfg: ChannelConfig) -> ChannelTarget:
    """Operator-sense (direction, level) -> firmware-convention ChannelTarget.

    direction 0 (release) always maps to the zero target, even for a disabled channel, so a
    configuration change can never make a held channel impossible to release.
    Inputs are assumed already validated by command_validation.validate_user_input.
    """
    if direction == 0:
        return ZERO_TARGET
    if not cfg.enabled:
        raise MappingError(f'channel {channel} is disabled in configuration')
    demand = min(1.0, level * cfg.max_demand)
    return ChannelTarget(direction * cfg.polarity, demand)
