"""Telemetry payload encode/decode and plausibility checks (pure Python, no ROS).

Payload layout (version 1, little-endian, 139 bytes) - see docs/serial_protocol.md:

    header  '<IBBIHII'  telemetry_seq u32, firmware_state u8, accepted_seq_valid u8,
                        last_accepted_cmd_seq u32, command_age_ms u16,
                        active_faults u32, latched_faults u32               (20 bytes)
    7 x channel record '<BHhiii' (channels M1..M7 in order, no channel id on the wire):
                        valid_flags u8, fault_flags u16, applied_output_permille i16,
                        encoder_counts i32, joint_angle_urad i32, current_ma i32  (17 bytes)

A value whose valid bit is clear is decoded as NaN (floats) or 0 (counts) so an invalid
reading can never look like a plausible zero.
"""
import math
import struct
from dataclasses import dataclass, field
from typing import List

from arm_hardware.serial_protocol import (NUM_CHANNELS, PROTOCOL_VERSION, TYPE_TELEMETRY, Frame)

HEADER_FMT = '<IBBIHII'
CHANNEL_FMT = '<BHhiii'
HEADER_LEN = struct.calcsize(HEADER_FMT)      # 20
CHANNEL_LEN = struct.calcsize(CHANNEL_FMT)    # 17
TELEMETRY_PAYLOAD_LEN = HEADER_LEN + NUM_CHANNELS * CHANNEL_LEN   # 139

# Mirrors of arm_interfaces constants (tests cross-check these against the .msg files).
VALID_OUTPUT, VALID_ENCODER, VALID_ANGLE, VALID_CURRENT = 1, 2, 4, 8
VALID_MASK = 0x0F
MOTOR_FAULT_MASK = 0x0F
GLOBAL_FAULT_MASK = 0xFF
FW_STATE_MAX = 6
COMMAND_AGE_UNKNOWN = 65535


class TelemetryError(ValueError):
    """Frame is well-formed at the framing level but not acceptable telemetry."""


@dataclass
class ChannelTelemetry:
    valid_flags: int = 0
    fault_flags: int = 0
    applied_output: float = math.nan     # signed, normalized [-1, 1]
    encoder_counts: int = 0              # motor-side raw counts
    joint_angle_rad: float = math.nan    # AS5600 joint angle
    current_a: float = math.nan          # amperes


@dataclass
class TelemetryData:
    telemetry_seq: int = 0
    firmware_state: int = 0
    accepted_seq_valid: bool = False
    last_accepted_cmd_seq: int = 0
    command_age_ms: int = COMMAND_AGE_UNKNOWN
    active_faults: int = 0
    latched_faults: int = 0
    channels: List[ChannelTelemetry] = field(
        default_factory=lambda: [ChannelTelemetry() for _ in range(NUM_CHANNELS)])
    version: int = PROTOCOL_VERSION


def build_telemetry_payload(data: TelemetryData) -> bytes:
    """Encode TelemetryData to a payload (used by simulators/tests; firmware does the same)."""
    if len(data.channels) != NUM_CHANNELS:
        raise TelemetryError(f'need {NUM_CHANNELS} channels, got {len(data.channels)}')
    out = struct.pack(HEADER_FMT, data.telemetry_seq & 0xFFFFFFFF, data.firmware_state,
                      1 if data.accepted_seq_valid else 0,
                      data.last_accepted_cmd_seq & 0xFFFFFFFF, data.command_age_ms,
                      data.active_faults, data.latched_faults)
    for ch in data.channels:
        vf = ch.valid_flags
        out += struct.pack(
            CHANNEL_FMT, vf, ch.fault_flags,
            int(round(ch.applied_output * 1000.0)) if vf & VALID_OUTPUT else 0,
            ch.encoder_counts if vf & VALID_ENCODER else 0,
            int(round(ch.joint_angle_rad * 1e6)) if vf & VALID_ANGLE else 0,
            int(round(ch.current_a * 1000.0)) if vf & VALID_CURRENT else 0)
    return out


def parse_telemetry_frame(frame: Frame) -> TelemetryData:
    """Validate and decode a telemetry Frame. Raises TelemetryError on any violation.

    A version other than PROTOCOL_VERSION raises TelemetryError with `.version_mismatch`
    set so the bridge can report PROTOCOL_MISMATCH instead of a generic malformed frame.
    """
    if frame.msg_type != TYPE_TELEMETRY:
        raise TelemetryError(f'unexpected message type 0x{frame.msg_type:02X}')
    if frame.version != PROTOCOL_VERSION:
        err = TelemetryError(f'protocol version {frame.version} != {PROTOCOL_VERSION}')
        err.version_mismatch = True       # type: ignore[attr-defined]
        err.remote_version = frame.version  # type: ignore[attr-defined]
        raise err
    if len(frame.payload) != TELEMETRY_PAYLOAD_LEN:
        raise TelemetryError(f'payload {len(frame.payload)} bytes, expected {TELEMETRY_PAYLOAD_LEN}')

    (seq, fw_state, acc_valid, acc_seq, age, active, latched) = struct.unpack_from(
        HEADER_FMT, frame.payload, 0)
    if fw_state > FW_STATE_MAX:
        raise TelemetryError(f'unknown firmware state {fw_state}')
    if acc_valid not in (0, 1):
        raise TelemetryError('accepted_seq_valid must be 0 or 1')
    if active & ~GLOBAL_FAULT_MASK or latched & ~GLOBAL_FAULT_MASK:
        raise TelemetryError('unknown global fault bits set')

    channels = []
    for i in range(NUM_CHANNELS):
        vf, ff, out_pm, enc, ang_urad, cur_ma = struct.unpack_from(
            CHANNEL_FMT, frame.payload, HEADER_LEN + i * CHANNEL_LEN)
        if vf & ~VALID_MASK:
            raise TelemetryError(f'channel {i + 1}: unknown valid_flags bits 0x{vf:02X}')
        if ff & ~MOTOR_FAULT_MASK:
            raise TelemetryError(f'channel {i + 1}: unknown fault_flags bits 0x{ff:04X}')
        if vf & VALID_OUTPUT and abs(out_pm) > 1000:
            raise TelemetryError(f'channel {i + 1}: applied output {out_pm} permille out of range')
        channels.append(ChannelTelemetry(
            valid_flags=vf, fault_flags=ff,
            applied_output=out_pm / 1000.0 if vf & VALID_OUTPUT else math.nan,
            encoder_counts=enc if vf & VALID_ENCODER else 0,
            joint_angle_rad=ang_urad / 1e6 if vf & VALID_ANGLE else math.nan,
            current_a=cur_ma / 1000.0 if vf & VALID_CURRENT else math.nan))
    return TelemetryData(telemetry_seq=seq, firmware_state=fw_state,
                         accepted_seq_valid=bool(acc_valid), last_accepted_cmd_seq=acc_seq,
                         command_age_ms=age, active_faults=active, latched_faults=latched,
                         channels=channels, version=frame.version)
