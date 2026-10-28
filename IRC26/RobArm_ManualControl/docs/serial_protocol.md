# Serial protocol (Jetson <-> STM32): version 1 — DRAFT PROPOSAL

**Status: draft, not frozen.** This is a proposal written so the ROS bridge and the STM32
firmware can be implemented against one document. It is **not** the STM32 specification
until the firmware developer and the team review and freeze it. The reference
implementation is `src/arm_hardware/arm_hardware/serial_protocol.py` and
`telemetry_parser.py`; the vectors in section 8 are generated from it and are the
conformance tests for firmware. Nothing here has been tested against real STM32 firmware.

Values marked **(default)** are development defaults, not hardware specifications.

## 1. Physical link

- USB serial to the NUCLEO-F446ZE. Note: the ST-LINK virtual COM port is a real UART
  (USART3) behind the ST-LINK, so the baud rate must match on both sides; a native-USB CDC
  port ignores it.
- 8 data bits, no parity, 1 stop bit, no flow control. Baud **115200 (default)**, configurable
  on the ROS side (`baudrate` parameter) and in firmware.
- The link is a byte stream with no guaranteed alignment; framing (section 2) restores it.

## 2. Framing

```
body  = version u8 | type u8 | len u8 | payload[len] | crc16 u16
frame = COBS(body) + 0x00
```

- **Byte order: little-endian** for every multi-byte field. Signed fields are two's complement.
- `len` = payload length, 0..200. The body is therefore at most 205 bytes. COBS only needs
  its 0xFF block form for 254 or more consecutive non-zero bytes, so **firmware does not need
  to implement the 0xFF block case** (reject any frame whose encoded size exceeds 207 bytes).
- **CRC-16/CCITT-FALSE**: polynomial 0x1021, init 0xFFFF, no input/output reflection, xorout
  0x0000, computed over `version | type | len | payload`, transmitted low byte first.
  Check value: CRC of the ASCII string `123456789` is **0x29B1**.
- **COBS** (Consistent Overhead Byte Stuffing) removes all 0x00 bytes from the body; the single
  0x00 after it is the frame delimiter. Receivers split on 0x00, COBS-decode each non-empty
  chunk, then check length and CRC. Empty chunks are ignored. After any error the receiver
  simply waits for the next 0x00 (this is the resynchronization mechanism).
- The outer layout (version, type, len, crc) is **fixed forever**; the `version` byte only
  governs payload meaning. `version` for this document is **1**.
- Receive-side rules for firmware: bound the receive buffer (discard until 0x00 on overflow);
  a frame failing COBS, length or CRC checks **must not** change any motor target and
  **must not** refresh the command watchdog.

## 3. Message types

| Type | Name | Direction | Payload |
|---|---|---|---|
| 0x01 | SETPOINT | host → firmware | `command_seq u32`, then 7 × `signed_permille i16` (M1..M7) = 18 bytes |
| 0x02 | STOP | host → firmware | `command_seq u32` = 4 bytes |
| 0x03 | ESTOP | host → firmware | `command_seq u32` = 4 bytes |
| 0x04 | REARM | host → firmware | `command_seq u32` = 4 bytes |
| 0x81 | TELEMETRY | firmware → host | 139 bytes, section 4 |

- `signed_permille`: sign = direction (positive = firmware's positive output direction, which
  is mechanically **uncalibrated**), magnitude = demand × 1000, range −1000..+1000. 0 means
  "ramp to zero". Values outside ±1000 invalidate the frame. A non-zero demand never encodes
  as 0 (tiny demands become ±1).
- **Firmware owns ramping.** SETPOINT carries only the target. The firmware moves its output
  toward it by its own periodic ramp generator; the host never sends ramp steps.
- STOP, ESTOP and REARM carry **no motor data** by design: a stop cannot be corrupted into
  motion by channel data.
- Unknown message types and wrong payload lengths: discard, count, do not refresh the watchdog.

## 4. Telemetry payload (type 0x81, 139 bytes)

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | u32 | `telemetry_seq` | Increments every telemetry frame; gaps = lost frames |
| 4 | u8 | `firmware_state` | 0 UNKNOWN, 1 BOOT, 2 IDLE, 3 ACTIVE, 4 CONTROLLED_STOP, 5 FAULT, 6 RECOVERY (>6 invalid) |
| 5 | u8 | `accepted_seq_valid` | 1 only if `last_accepted_cmd_seq` is meaningful; must be 0 or 1 |
| 6 | u32 | `last_accepted_cmd_seq` | `command_seq` of the last command that **passed firmware validation and was applied** |
| 10 | u16 | `command_age_ms` | Time since last valid command; saturates at 65534; 65535 = unknown |
| 12 | u32 | `active_faults` | Bits below; unknown bits invalidate the frame |
| 16 | u32 | `latched_faults` | Same bits; cleared only by an accepted REARM |
| 20 | 7 × 17 | channel records M1..M7 in order | no channel id on the wire |

Global fault bits: 0x01 COMMAND_TIMEOUT, 0x02 ESTOP_LATCHED, 0x04 SENSOR_INVALID,
0x08 GRIPPER_OVERCURRENT, 0x10 GRIPPER_MAX_RUNTIME, 0x20 DRIVER, 0x40 FIRMWARE_WATCHDOG,
0x80 PROTOCOL_ERROR.

Channel record (17 bytes):

| Offset | Size | Field | Unit |
|---|---|---|---|
| 0 | u8 | `valid_flags` | bit0 OUTPUT, bit1 ENCODER, bit2 ANGLE, bit3 CURRENT (higher bits invalid) |
| 1 | u16 | `fault_flags` | bit0 SENSOR_INVALID, bit1 OVERCURRENT_STALL, bit2 MAX_RUNTIME, bit3 DRIVER |
| 3 | i16 | `applied_output_permille` | post-ramp output actually applied, −1000..1000 |
| 5 | i32 | `encoder_counts` | raw motor-side counts |
| 9 | i32 | `joint_angle_urad` | AS5600 joint angle in micro-radians, after firmware calibration |
| 13 | i32 | `current_ma` | milliamps |

A field whose valid bit is clear must be sent as 0 and is **ignored** by the host (it
becomes NaN/absent on the ROS side, never a plausible zero). Which channels carry which
sensor: M1, M4, M5, M6 → ENCODER; M2, M3 → ANGLE; M7 → CURRENT; all → OUTPUT.

## 5. Behavioural contract (what the host assumes; firmware must provide)

1. **Heartbeat.** The host sends a SETPOINT (or STOP/ESTOP/REARM) at a fixed rate
   (**20 Hz (default)**) whenever the link is up. Every *valid* command refreshes the
   firmware command watchdog.
2. **Command timeout (firmware-owned, value TBD).** If no valid command arrives within the
   timeout, the firmware applies its communication-loss stop policy locally, reports
   `COMMAND_TIMEOUT`, and does **not** resume on its own when commands return unless that is
   explicitly specified and tested. Suggested starting point: at least 3 command periods.
3. **Acceptance.** `last_accepted_cmd_seq` + `accepted_seq_valid` is the only acceptance
   signal. Transmitted ≠ received ≠ accepted ≠ moved (see `ros_interface_spec.md` §3).
4. **Priority.** ESTOP must be handled with priority over any other queued command and
   latches until an accepted REARM. While latched, SETPOINTs are rejected (not accepted).
5. **REARM** clears a latched ESTOP/fault only if the firmware's own conditions allow; it
   never starts motion. Outputs stay at zero until a new SETPOINT with non-zero demand.
6. **Telemetry rate: 20 Hz (default)**, sent regardless of commands so the host can detect
   a stale link. Estimated size 146 bytes/frame ≈ 2.9 kB/s ≈ 25 % of a 115200-baud link.
7. **Version.** The host sends `version = 1`. A firmware receiving another
   version must not act on SETPOINTs. The host treats telemetry with another version as
   `PROTOCOL_MISMATCH` and sends no SETPOINTs (STOP/ESTOP are still sent, best effort).
8. **Boot/reset.** Outputs start at zero and stay there until a valid non-zero SETPOINT is
   received after boot; the host additionally refuses to send non-zero SETPOINTs until it has
   seen healthy telemetry and a zero command after every (re)connect.

## 6. Units and encoding summary

Angles: micro-radians on the wire, radians in ROS. Current: milliamps → amperes. Output:
per-mille → fraction. Encoder: raw counts (not joint angle). Sequence numbers: u32, wrap.

## 7. Open protocol decisions (not guessed)

Baud rate and telemetry/command rates (defaults above); command timeout and its policy;
ESTOP output behaviour (coast / brake / disable); fault latching and REARM conditions;
whether the AS5600 zero offset/direction calibration lives in firmware (assumed yes);
whether telemetry should also be sent event-driven on fault changes.

## 8. Test vectors

Reference values for `version = 1`. Firmware parsers and encoders must match byte for byte.

CRC-16/CCITT-FALSE of ASCII `123456789`: `0x29B1`.

COBS (raw → encoded): `` → `01`; `00` → `01 01`; `00 00` → `01 01 01`; `00 11 00` → `01 02 11 01`;
`11 22 00 33` → `03 11 22 02 33`; `11 22 33 44` → `05 11 22 33 44`; `11 00 00 00` → `02 11 01 01 01`.

Commands (complete frames including the trailing `00`):

| Case | Frame (hex) |
|---|---|
| STOP, seq 1 | `05 01 02 04 01 01 01 03 5d 3d 00` |
| ESTOP, seq 2 | `05 01 03 04 02 01 01 03 21 e3 00` |
| REARM, seq 3 | `05 01 04 04 03 01 01 03 d4 5d 00` |
| SETPOINT, seq 0x01020304, all channels zero | `08 01 01 12 04 03 02 01 01 01 01 01 01 01 01 01 01 01 01 01 01 03 ca 64 00` |
| SETPOINT, seq 5, M1 +0.100, M2 −0.250, M7 +1.000 (permille 100, −250, 0,0,0,0, 1000) | `05 01 01 12 05 01 01 02 64 03 06 ff 01 01 01 01 01 01 01 05 e8 03 d0 1b 00` |

Decoded body of the STOP frame: `01 02 04 | 01 00 00 00 | 5d 3d` (version, type, len, seq=1, CRC 0x3D5D).

Telemetry example (146 bytes with delimiter). Content: seq 1000, ACTIVE, accepted_seq_valid
with last accepted 5, command_age 40 ms, no faults; M1 output +0.100 and encoder 1234;
M2 output −0.250 and angle 1.2345 rad; M3–M6 output 0; M7 output +0.500 and current 0.750 A.

Body before COBS (144 bytes):

```
01 81 8b e8 03 00 00 03 01 05 00 00 00 28 00 00 00 00 00 00 00 00 00 03 00 00 64 00 d2 04 00 00 00 00 00 00 00 00 00 00 05 00 00 06 ff 00 00 00
00 44 d6 12 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 09 00 00 f4 01 00 00 00 00 00 00 00 00 ee 02 00 00 b6 ef
```

Full frame on the wire (146 bytes):

```
06 01 81 8b e8 03 01 04 03 01 05 01 01 02 28 01 01 01 01 01 01 01 01 02 03 01 02 64 03 d2 04 01 01 01 01 01 01 01 01 01 02 05 01 03 06 ff 01 01
01 04 44 d6 12 01 01 01 01 02 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 02 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 02 01 01 01 01
01 01 01 01 01 01 01 01 01 01 01 01 02 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 02 09 01 03 f4 01 01 01 01 01 01 01 01 03 ee 02 01 03 b6
ef 00
```
