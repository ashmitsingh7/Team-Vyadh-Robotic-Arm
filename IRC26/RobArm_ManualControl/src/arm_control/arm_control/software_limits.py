"""ROS-side software limits (pure Python).

These checks are an extra layer and NEVER replace the STM32's independent safety checks.
Joint limits are disabled by default because no limits or calibration exist yet.
"""
import math
from typing import Optional

from arm_control.control_config import ChannelConfig


def check_motion(channel: int, direction: int, cfg: ChannelConfig,
                 angle_rad: Optional[float]) -> Optional[str]:
    """Return a reason string if moving `channel` in firmware `direction` must be refused,
    else None. A zero direction (stop/release) is never refused.

    angle_rad is the latest VALID AS5600 joint angle for the channel, or None/NaN when
    unavailable. With joint limits enabled, an unavailable angle blocks motion: the limit
    cannot be verified, so it is not assumed to be satisfied.
    """
    if direction == 0:
        return None
    if not cfg.enabled:
        return f'M{channel} disabled in configuration'
    if not cfg.joint_limits_enabled:
        return None
    if angle_rad is None or not math.isfinite(angle_rad):
        return f'M{channel} joint angle unavailable; cannot verify joint limits'
    effective = direction * cfg.angle_sign   # sign of the resulting change in joint angle
    if effective < 0 and angle_rad <= cfg.joint_min_rad:
        return f'M{channel} at lower joint limit ({angle_rad:.3f} rad <= {cfg.joint_min_rad:.3f})'
    if effective > 0 and angle_rad >= cfg.joint_max_rad:
        return f'M{channel} at upper joint limit ({angle_rad:.3f} rad >= {cfg.joint_max_rad:.3f})'
    return None
