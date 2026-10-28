"""Serial protocol v1 (DRAFT): framing, CRC and command encoding. Pure Python, no ROS.

Specification: docs/serial_protocol.md - that document is the contract; this
file implements it. Nothing here is the STM32 specification until the firmware developer
and the team have agreed and frozen it.

Wire format (all multi-byte fields little-endian):

    body  = version u8 | type u8 | len u8 | payload[len] | crc16 u16
    frame = COBS(body) + 0x00

CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0) over
version..payload. The outer layout (version, type, len, crc) never changes between
protocol versions; the version byte only governs payload semantics.
"""
import struct
from dataclasses import dataclass
from typing import List, Optional, Union

PROTOCOL_VERSION = 1

# Host -> firmware message types.
TYPE_SETPOINT = 0x01
TYPE_STOP = 0x02
TYPE_ESTOP = 0x03
TYPE_REARM = 0x04
# Firmware -> host message types.
TYPE_TELEMETRY = 0x81

HOST_TYPES = (TYPE_SETPOINT, TYPE_STOP, TYPE_ESTOP, TYPE_REARM)

NUM_CHANNELS = 7
MAX_PAYLOAD = 200
# Body is at most 3 + 200 + 2 = 205 bytes (< 254), so COBS never needs an 0xFF block.
MAX_BODY = 3 + MAX_PAYLOAD + 2
MAX_ENCODED = MAX_BODY + 2  # COBS adds at most one byte per 254 bytes; keep a margin

SETPOINT_PAYLOAD_LEN = 4 + 2 * NUM_CHANNELS   # seq u32 + 7 x int16 permille
SEQ_ONLY_PAYLOAD_LEN = 4                      # STOP / ESTOP / REARM carry only the sequence

_COMMAND_TYPE_TO_WIRE = {1: TYPE_SETPOINT, 2: TYPE_STOP, 3: TYPE_ESTOP, 4: TYPE_REARM}


class ProtocolError(ValueError):
    """Raised for anything that cannot be encoded/decoded according to the protocol."""


# ---------------------------------------------------------------- CRC and COBS ---------

def crc16_ccitt_false(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE. Check value: crc16_ccitt_false(b'123456789') == 0x29B1."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing: output contains no 0x00 bytes."""
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(encoded: bytes) -> bytes:
    """Inverse of cobs_encode. `encoded` excludes the 0x00 frame delimiter."""
    out = bytearray()
    i = 0
    n = len(encoded)
    while i < n:
        code = encoded[i]
        if code == 0:
            raise ProtocolError('zero byte inside COBS data')
        i += 1
        end = i + code - 1
        if end > n:
            raise ProtocolError('COBS block runs past end of data')
        chunk = encoded[i:end]
        if 0 in chunk:
            raise ProtocolError('zero byte inside COBS block')
        out += chunk
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------- frames -------------

@dataclass(frozen=True)
class Frame:
    version: int
    msg_type: int
    payload: bytes


@dataclass(frozen=True)
class FrameError:
    """A chunk between delimiters that could not be accepted. kind is one of:
    'overflow', 'cobs', 'length', 'crc'."""
    kind: str
    detail: str = ''


def encode_frame(msg_type: int, payload: bytes, version: int = PROTOCOL_VERSION) -> bytes:
    """Build a complete on-the-wire frame (COBS-encoded body followed by 0x00)."""
    if not 0 <= msg_type <= 0xFF or not 0 <= version <= 0xFF:
        raise ProtocolError('type/version must fit in one byte')
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f'payload {len(payload)} bytes exceeds {MAX_PAYLOAD}')
    head = bytes([version, msg_type, len(payload)]) + bytes(payload)
    body = head + struct.pack('<H', crc16_ccitt_false(head))
    return cobs_encode(body) + b'\x00'


def decode_body(body: bytes) -> Frame:
    """Validate length and CRC of an already COBS-decoded body."""
    if len(body) < 5:
        raise ProtocolError('body shorter than 5 bytes')
    version, msg_type, length = body[0], body[1], body[2]
    if len(body) != 3 + length + 2:
        raise ProtocolError(f'length field {length} inconsistent with body size {len(body)}')
    (crc_rx,) = struct.unpack_from('<H', body, len(body) - 2)
    if crc16_ccitt_false(body[:-2]) != crc_rx:
        raise ProtocolError('CRC mismatch')
    return Frame(version, msg_type, bytes(body[3:3 + length]))


class FrameDecoder:
    """Incremental stream decoder: feed arbitrary byte chunks, get frames and errors.

    Handles partial frames (buffered until the delimiter), corrupt frames (reported, stream
    resynchronizes at the next 0x00) and runaway garbage (buffer is bounded; excess data is
    discarded until the next delimiter and reported once as 'overflow'). Empty chunks
    (back-to-back delimiters) are ignored silently.
    """

    def __init__(self, max_encoded: int = MAX_ENCODED):
        self._buf = bytearray()
        self._overflow = False
        self._max = max_encoded

    def feed(self, data: bytes) -> List[Union[Frame, FrameError]]:
        results: List[Union[Frame, FrameError]] = []
        for byte in data:
            if byte != 0:
                if self._overflow:
                    continue
                if len(self._buf) >= self._max:
                    self._overflow = True
                    self._buf.clear()
                    continue
                self._buf.append(byte)
                continue
            # delimiter
            if self._overflow:
                results.append(FrameError('overflow', f'more than {self._max} bytes without delimiter'))
                self._overflow = False
                self._buf.clear()
                continue
            if not self._buf:
                continue
            chunk = bytes(self._buf)
            self._buf.clear()
            try:
                body = cobs_decode(chunk)
            except ProtocolError as exc:
                results.append(FrameError('cobs', str(exc)))
                continue
            try:
                results.append(decode_body(body))
            except ProtocolError as exc:
                kind = 'crc' if 'CRC' in str(exc) else 'length'
                results.append(FrameError(kind, str(exc)))
        return results


# ---------------------------------------------------------------- commands -----------

def _signed_permille(direction: int, demand: float) -> int:
    """direction (-1/0/+1) and demand [0,1] -> signed per-mille. Non-zero demand never
    quantizes to zero (it becomes 1), so a requested movement is never silently a stop."""
    if direction == 0:
        return 0
    permille = max(1, min(1000, int(round(demand * 1000.0))))
    return permille * direction


def encode_command(command) -> bytes:
    """Encode a validated ArmCommand (or duck-typed equivalent) into a wire frame.

    Raises ProtocolError if the command is invalid. This is the last defence; the bridge
    validates first and counts rejections.
    """
    from arm_control.command_validation import validate_arm_command  # shared validator

    errors = validate_arm_command(command)
    if errors:
        raise ProtocolError('invalid ArmCommand: ' + '; '.join(errors))
    wire_type = _COMMAND_TYPE_TO_WIRE[command.command_type]
    seq = int(command.sequence) & 0xFFFFFFFF
    payload = struct.pack('<I', seq)
    if wire_type == TYPE_SETPOINT:
        values = [0] * NUM_CHANNELS
        for motor in command.motors:
            values[motor.channel - 1] = _signed_permille(motor.direction, motor.demand)
        payload += struct.pack('<7h', *values)
    return encode_frame(wire_type, payload)


@dataclass(frozen=True)
class DecodedCommand:
    """Host->firmware command as a firmware would see it (used by simulators and tests)."""
    msg_type: int
    sequence: int
    permille: Optional[List[int]]   # 7 signed values for SETPOINT, else None


def decode_command_frame(frame: Frame) -> DecodedCommand:
    """Parse a host->firmware frame (what the STM32 parser must do). Strict on length/type."""
    if frame.msg_type not in HOST_TYPES:
        raise ProtocolError(f'unknown command type 0x{frame.msg_type:02X}')
    if frame.msg_type == TYPE_SETPOINT:
        if len(frame.payload) != SETPOINT_PAYLOAD_LEN:
            raise ProtocolError('SETPOINT payload length wrong')
        seq = struct.unpack_from('<I', frame.payload, 0)[0]
        values = list(struct.unpack_from('<7h', frame.payload, 4))
        if any(abs(v) > 1000 for v in values):
            raise ProtocolError('SETPOINT permille out of range')
        return DecodedCommand(frame.msg_type, seq, values)
    if len(frame.payload) != SEQ_ONLY_PAYLOAD_LEN:
        raise ProtocolError('STOP/ESTOP/REARM payload length wrong')
    return DecodedCommand(frame.msg_type, struct.unpack('<I', frame.payload)[0], None)
