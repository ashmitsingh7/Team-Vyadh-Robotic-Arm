"""Configuration data classes for arm_control (pure Python, no ROS imports).

Every default here is an UNCALIBRATED development placeholder. The authoritative values
come from arm_bringup/config/*.yaml. Nothing here has been verified on hardware, and these
ROS-side limits never replace the STM32's independent safety checks.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List

from arm_control.channels import CHANNEL_COUNT

# Mirrors of ArmState.FAULT_* bits (tests cross-check these against the .msg file).
FAULT_COMMAND_TIMEOUT = 1
FAULT_ESTOP_LATCHED = 2
FAULT_SENSOR_INVALID = 4
FAULT_GRIPPER_OVERCURRENT = 8
FAULT_GRIPPER_MAX_RUNTIME = 16
FAULT_DRIVER = 32
FAULT_FIRMWARE_WATCHDOG = 64
FAULT_PROTOCOL_ERROR = 128

# Faults that make the ROS side refuse NEW motion (placeholder policy, to be agreed with the
# firmware developer). COMMAND_TIMEOUT is deliberately not included: it clears only when
# valid commands flow again, so blocking on it would deadlock recovery.
DEFAULT_BLOCKING_FAULTS_MASK = FAULT_ESTOP_LATCHED | FAULT_DRIVER | FAULT_FIRMWARE_WATCHDOG


@dataclass
class ChannelConfig:
    polarity: int = 1                  # +1/-1, UNCALIBRATED: operator "positive" -> firmware "positive"
    max_demand: float = 0.20           # UNCALIBRATED fraction of firmware full scale, (0, 1]
    enabled: bool = True
    joint_limits_enabled: bool = False
    joint_min_rad: float = 0.0         # UNKNOWN until calibrated
    joint_max_rad: float = 0.0         # UNKNOWN until calibrated
    angle_sign: int = 1                # UNCALIBRATED: +1 if +firmware direction increases the AS5600 angle


@dataclass
class ControlConfig:
    channels: Dict[int, ChannelConfig] = field(
        default_factory=lambda: {n: ChannelConfig() for n in range(1, CHANNEL_COUNT + 1)})
    command_rate_hz: float = 20.0
    input_timeout_s: float = 0.3       # backup deadman: a MOVE not refreshed within this expires
    require_fresh_state: bool = True   # refuse new motion without fresh telemetry/healthy link
    state_stale_after_s: float = 0.5
    diag_stale_after_s: float = 5.0
    blocking_faults_mask: int = DEFAULT_BLOCKING_FAULTS_MASK
    rearm_repeat: int = 5              # consecutive REARM commands published after an operator REARM
    stop_repeat: int = 3               # consecutive STOP commands published after an operator STOP


def validate_config(cfg: ControlConfig) -> List[str]:
    """Return a list of configuration problems (empty = acceptable)."""
    problems: List[str] = []
    if sorted(cfg.channels) != list(range(1, CHANNEL_COUNT + 1)):
        problems.append(f'channel config must cover exactly 1..{CHANNEL_COUNT}, got {sorted(cfg.channels)}')
    for n, ch in sorted(cfg.channels.items()):
        if ch.polarity not in (-1, 1):
            problems.append(f'm{n}.polarity must be +1 or -1, got {ch.polarity!r}')
        if not 0.0 < ch.max_demand <= 1.0:
            problems.append(f'm{n}.max_demand must be in (0, 1], got {ch.max_demand!r}')
        if ch.angle_sign not in (-1, 1):
            problems.append(f'm{n}.angle_sign must be +1 or -1, got {ch.angle_sign!r}')
        if ch.joint_limits_enabled and not ch.joint_min_rad < ch.joint_max_rad:
            problems.append(f'm{n}: joint_min_rad must be < joint_max_rad when joint limits are enabled')
    if not 1.0 <= cfg.command_rate_hz <= 200.0:
        problems.append('command_rate_hz must be in [1, 200]')
    if cfg.input_timeout_s <= 1.0 / cfg.command_rate_hz:
        problems.append('input_timeout_s must exceed one command period')
    if cfg.state_stale_after_s <= 0 or cfg.diag_stale_after_s <= 0:
        problems.append('stale thresholds must be positive')
    if cfg.rearm_repeat < 1 or cfg.stop_repeat < 1:
        problems.append('rearm_repeat and stop_repeat must be >= 1')
    return problems


def load_config(get: Callable[[str], object]) -> ControlConfig:
    """Build a ControlConfig from a parameter getter `get(name) -> value`.

    The node passes a function reading declared ROS parameters; tests pass a dict lookup.
    Parameter names: command_rate_hz, ..., channels.m<N>.polarity, etc.
    """
    cfg = ControlConfig()
    cfg.command_rate_hz = float(get('command_rate_hz'))
    cfg.input_timeout_s = float(get('input_timeout_s'))
    cfg.require_fresh_state = bool(get('require_fresh_state'))
    cfg.state_stale_after_s = float(get('state_stale_after_s'))
    cfg.diag_stale_after_s = float(get('diag_stale_after_s'))
    cfg.blocking_faults_mask = int(get('blocking_faults_mask'))
    cfg.rearm_repeat = int(get('rearm_repeat'))
    cfg.stop_repeat = int(get('stop_repeat'))
    for n in range(1, CHANNEL_COUNT + 1):
        p = f'channels.m{n}.'
        cfg.channels[n] = ChannelConfig(
            polarity=int(get(p + 'polarity')),
            max_demand=float(get(p + 'max_demand')),
            enabled=bool(get(p + 'enabled')),
            joint_limits_enabled=bool(get(p + 'joint_limits_enabled')),
            joint_min_rad=float(get(p + 'joint_min_rad')),
            joint_max_rad=float(get(p + 'joint_max_rad')),
            angle_sign=int(get(p + 'angle_sign')))
    return cfg
