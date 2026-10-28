"""Convert pure-Python telemetry/diagnostic data into ROS messages (shared by both bridges).

The only module of the bridge logic that imports arm_interfaces. Float fields are always
passed as Python floats (the generated message classes reject ints for float fields).
"""
from arm_interfaces.msg import ArmDiagnostics, ArmState, MotorState

from arm_hardware.bridge_types import (LINK_CONNECTED, LINK_CONNECTING, LINK_DISCONNECTED,  # noqa: F401
                                       LINK_PROTOCOL_MISMATCH, LINK_TELEMETRY_STALE, BridgeStats)
from arm_hardware.serial_protocol import PROTOCOL_VERSION
from arm_hardware.telemetry_parser import TelemetryData

def telemetry_to_state_msg(data: TelemetryData, stamp, simulated: bool) -> ArmState:
    msg = ArmState()
    msg.header.stamp = stamp
    msg.simulated = bool(simulated)
    msg.firmware_state = int(data.firmware_state)
    msg.telemetry_sequence = int(data.telemetry_seq)
    msg.accepted_sequence_valid = bool(data.accepted_seq_valid)
    msg.last_accepted_command_sequence = int(data.last_accepted_cmd_seq)
    msg.command_age_ms = int(data.command_age_ms)
    msg.active_faults = int(data.active_faults)
    motors = []
    for i, ch in enumerate(data.channels):
        m = MotorState()
        m.channel = i + 1
        m.valid_flags = int(ch.valid_flags)
        m.applied_output = float(ch.applied_output)
        m.encoder_counts = int(ch.encoder_counts)
        m.joint_angle_rad = float(ch.joint_angle_rad)
        m.current_a = float(ch.current_a)
        m.fault_flags = int(ch.fault_flags)
        motors.append(m)
    msg.motors = motors
    return msg


def stats_to_diagnostics_msg(stats: BridgeStats, stamp) -> ArmDiagnostics:
    msg = ArmDiagnostics()
    msg.header.stamp = stamp
    msg.simulated = bool(stats.simulated)
    msg.link_state = int(stats.link_state)
    msg.local_protocol_version = PROTOCOL_VERSION
    msg.remote_version_valid = bool(stats.remote_version_valid)
    msg.remote_protocol_version = int(stats.remote_protocol_version)
    msg.last_telemetry_age_s = float(stats.last_telemetry_age_s)
    for name in ('commands_received', 'commands_rejected', 'frames_transmitted',
                 'frames_received_valid', 'frames_crc_error', 'frames_malformed',
                 'telemetry_timeouts', 'reconnect_count', 'latched_faults'):
        setattr(msg, name, int(getattr(stats, name)) & 0xFFFFFFFF)
    return msg
