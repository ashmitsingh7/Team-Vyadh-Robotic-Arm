"""Telemetry freshness and health assessment (pure Python, no ROS imports).

Used by manual_control_node (to refuse new motion without healthy feedback) and by
arm_state_monitor_node (operator-facing reports). Stale or simulated data is always labelled.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from arm_control.channels import CHANNEL_COUNT, CHANNEL_NAMES
from arm_control.control_config import (FAULT_COMMAND_TIMEOUT, FAULT_DRIVER, FAULT_ESTOP_LATCHED,
                                        FAULT_FIRMWARE_WATCHDOG, FAULT_GRIPPER_MAX_RUNTIME,
                                        FAULT_GRIPPER_OVERCURRENT, FAULT_PROTOCOL_ERROR,
                                        FAULT_SENSOR_INVALID)

LINK_CONNECTED = 2
VALID_OUTPUT, VALID_ENCODER, VALID_ANGLE, VALID_CURRENT = 1, 2, 4, 8
FW_STATE_NAMES = {0: 'UNKNOWN', 1: 'BOOT', 2: 'IDLE', 3: 'ACTIVE', 4: 'CONTROLLED_STOP',
                  5: 'FAULT', 6: 'RECOVERY'}
LINK_NAMES = {0: 'DISCONNECTED', 1: 'CONNECTING', 2: 'CONNECTED', 3: 'TELEMETRY_STALE',
              4: 'PROTOCOL_MISMATCH'}
FAULT_NAMES = {FAULT_COMMAND_TIMEOUT: 'COMMAND_TIMEOUT', FAULT_ESTOP_LATCHED: 'ESTOP_LATCHED',
               FAULT_SENSOR_INVALID: 'SENSOR_INVALID', FAULT_GRIPPER_OVERCURRENT: 'GRIPPER_OVERCURRENT',
               FAULT_GRIPPER_MAX_RUNTIME: 'GRIPPER_MAX_RUNTIME', FAULT_DRIVER: 'DRIVER',
               FAULT_FIRMWARE_WATCHDOG: 'FIRMWARE_WATCHDOG', FAULT_PROTOCOL_ERROR: 'PROTOCOL_ERROR'}
# Firmware states in which the ROS side will not issue new motion.
NO_MOTION_FW_STATES = (0, 1, 5, 6)


def fault_names(bits: int) -> List[str]:
    return [name for bit, name in sorted(FAULT_NAMES.items()) if bits & bit]


@dataclass
class StateAssessment:
    fresh: bool = False
    age_s: float = math.inf
    simulated: bool = False
    firmware_state: int = 0
    active_faults: int = 0
    link_state: Optional[int] = None
    blocked_reason: Optional[str] = None    # None => new motion permitted by state/link health
    angles: Dict[int, Optional[float]] = field(default_factory=dict)   # channel -> valid angle or None


def assess(state, state_rx_time: Optional[float], diag, diag_rx_time: Optional[float],
           now: float, state_stale_after_s: float, diag_stale_after_s: float,
           blocking_faults_mask: int) -> StateAssessment:
    """Evaluate the latest ArmState/ArmDiagnostics (duck-typed; either may be None).

    Times are on one monotonic clock (seconds). Freshness uses the time THIS node received
    the message, not the bridge's stamp, so a stalled publisher is detected.
    """
    a = StateAssessment()
    if state is not None and state_rx_time is not None:
        a.age_s = max(0.0, now - state_rx_time)
        a.fresh = a.age_s <= state_stale_after_s
        a.simulated = bool(state.simulated)
        a.firmware_state = int(state.firmware_state)
        a.active_faults = int(state.active_faults)
        for m in state.motors:
            ok = bool(m.valid_flags & VALID_ANGLE) and math.isfinite(m.joint_angle_rad)
            a.angles[int(m.channel)] = float(m.joint_angle_rad) if ok else None
    diag_fresh = False
    if diag is not None and diag_rx_time is not None:
        diag_fresh = (now - diag_rx_time) <= diag_stale_after_s
        a.link_state = int(diag.link_state)

    if state is None:
        a.blocked_reason = 'no telemetry received yet'
    elif not a.fresh:
        a.blocked_reason = f'telemetry stale ({a.age_s:.2f} s old)'
    elif diag is None or not diag_fresh:
        a.blocked_reason = 'no recent bridge diagnostics'
    elif a.link_state != LINK_CONNECTED:
        a.blocked_reason = f'link state {LINK_NAMES.get(a.link_state, a.link_state)}'
    elif a.firmware_state in NO_MOTION_FW_STATES:
        a.blocked_reason = f'firmware state {FW_STATE_NAMES.get(a.firmware_state)}'
    elif a.active_faults & blocking_faults_mask:
        a.blocked_reason = 'firmware faults: ' + ','.join(fault_names(a.active_faults & blocking_faults_mask))
    return a


def format_summary(state, a: StateAssessment) -> str:
    """One multi-line operator report. Marks SIMULATED and STALE data explicitly."""
    tags = []
    if a.simulated:
        tags.append('SIMULATED DATA')
    if not a.fresh:
        tags.append('STALE')
    head = (f"[{' | '.join(tags) if tags else 'live'}] fw={FW_STATE_NAMES.get(a.firmware_state, '?')} "
            f"link={LINK_NAMES.get(a.link_state, 'unknown')} age={a.age_s:.2f}s "
            f"faults={','.join(fault_names(a.active_faults)) or 'none'}")
    lines = [head]
    if state is not None:
        for m in sorted(state.motors, key=lambda x: x.channel):
            parts = [f'out={m.applied_output:+.2f}' if m.valid_flags & VALID_OUTPUT else 'out=n/a']
            if m.valid_flags & VALID_ENCODER:
                parts.append(f'enc={m.encoder_counts}')
            if m.valid_flags & VALID_ANGLE:
                parts.append(f'angle={m.joint_angle_rad:.3f}rad')
            if m.valid_flags & VALID_CURRENT:
                parts.append(f'I={m.current_a:.2f}A')
            if m.fault_flags:
                parts.append(f'faultbits=0x{m.fault_flags:X}')
            lines.append(f'  {CHANNEL_NAMES[m.channel - 1]}: ' + ' '.join(parts))
    return '\n'.join(lines)
