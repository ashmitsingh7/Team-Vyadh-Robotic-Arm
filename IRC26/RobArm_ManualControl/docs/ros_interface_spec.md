# ROS 2 interface specification (arm_interfaces) — v0.1 DRAFT

Status: **proposal for review; not frozen.** Nothing here has been built with `colcon`
or exercised on hardware. The serial wire protocol is specified separately in
`serial_protocol.md` (also a draft). This document must be agreed
before ROS and firmware implementations diverge.

## 1. Topics

| Topic | Type | Direction | Rate | Meaning |
|---|---|---|---|---|
| `/arm/user_input` | `UserInput` | teleop → control | on operator action (+ hold refresh) | Operator **intent** |
| `/arm/command` | `ArmCommand` | control → bridge | fixed heartbeat rate (config, TBD) | Requested full-arm **setpoint** |
| `/arm/state` | `ArmState` | bridge → monitor/control | per telemetry frame | What the firmware **reported** |
| `/arm/diagnostics` | `ArmDiagnostics` | bridge → monitor | low rate + on link change | Link/bridge health |

Mock and serial bridges publish and subscribe to exactly these topics. Mock output has
`simulated = true`.

## 2. Command semantics (decision)

**`ArmCommand` is a full seven-channel snapshot, never a per-channel update.**

Why: with a snapshot, a lost, late or malformed message can never leave one channel
"inheriting" an old target. The receiver always knows the complete requested state, and a
channel that should not move is sent explicitly as `(DIRECTION_STOP, 0.0)`.

A command is **valid** only if all of these hold. If any fails, the **whole** command is
rejected (never partially applied, never silently clamped), counted in
`ArmDiagnostics.commands_rejected`, and not transmitted:

1. `command_type` is one of SETPOINT / STOP / ESTOP / REARM (0 = INVALID is rejected).
2. `motors` has exactly 7 entries; channels 1..7 each appear exactly once (any order).
3. Every `MotorCommand` is valid: `direction ∈ {-1,0,+1}`, `demand` finite and in `[0,1]`,
   and `demand == 0 ⇔ direction == 0`.
4. For REARM every entry is `(0, 0.0)`.

**Safety exception:** STOP and ESTOP are honoured on `command_type` + `sequence` alone. Their
`motors` content is ignored and never transmitted (the wire frames carry no motor data), so
malformed channel data can neither block a stop nor cause motion. `UserInput` STOP, ESTOP and
REARM are likewise honoured on `action` alone.

Safe defaults: a default-constructed `ArmCommand` has `command_type = INVALID` and an empty
`motors` list (rejected); a default `MotorCommand` is `(channel 0, STOP, 0.0)` (channel 0
rejected); a default `UserInput` has `action = INVALID` (rejected).

### 2.1 STOP / ESTOP / REARM and the seven-channel state

| Type | Motors array | Intended effect (firmware owns the physical response) | After it |
|---|---|---|---|
| SETPOINT | any valid snapshot | Firmware ramps each channel toward the requested target | Must be re-sent as a heartbeat |
| STOP | ignored (arm_control sends zeros) | **Normal stop**: firmware controlled-deceleration policy | Next SETPOINT may move again |
| ESTOP | ignored (arm_control sends zeros) | **Emergency-stop request**: firmware emergency policy (coast/brake/disable = TBD, firmware-owned). Latched. | No SETPOINT accepted until REARM |
| REARM | all zero | Request to clear a latched ESTOP/fault if the firmware allows | Outputs stay zero; motion needs a **new** SETPOINT |

Rules (ROS side; the STM32 must independently enforce its own):

- ESTOP latches in `arm_control`. While latched, it publishes ESTOP/zero commands and
  refuses SETPOINTs. REARM requires an explicit operator action (`ACTION_REARM`).
- Reconnection, node restart or telemetry recovery **never** re-arms or resumes motion.
- A **software ESTOP is only a request over a software link.** It is not a physical
  emergency stop and does not replace one. Physical output safety and the command
  timeout live on the STM32.
- Four separate concepts that must not be merged: **normal stop** (STOP), **emergency-stop
  request** (ESTOP), **communication-loss response** (firmware timeout, reported as
  `FAULT_COMMAND_TIMEOUT`), **fault response** (firmware-detected, reported in fault bits).

## 3. Transmitted vs received vs accepted vs observed

| Fact | Where it is known | Reported as |
|---|---|---|
| Published | `arm_control` | `ArmCommand.sequence` |
| Transmitted | bridge handed the frame to the serial port | `frames_transmitted` counter only |
| Received | firmware got a framed message | **not claimed** unless the protocol says so |
| Accepted | firmware validated it | `ArmState.last_accepted_command_sequence`, **only if** `accepted_sequence_valid` (protocol must confirm explicitly) |
| Motion observed | sensor feedback changed | `MotorState` fields with their `valid_flags` |

No separate "ack" topic is created. `applied_output` is the firmware's ramped output, not
proof that the mechanism moved.

## 4. Field reference

Angles are **radians**, currents **amperes**, counts raw integers, output/demand
**normalized fractions** (not PWM duty or raw timer values). `std_msgs/Header` is used in
every top-level message; `frame_id` is unused.

### UserInput
| Field | Type | Unit / range | Notes |
|---|---|---|---|
| `header` | Header | ROS time | Time generated by teleop |
| `sequence` | uint32 | counter, wraps | Logging only |
| `action` | uint8 | 0 INVALID, 1 MOVE, 2 STOP, 3 ESTOP, 4 REARM | 0 rejected |
| `channel` | uint8 | 0 none; 1..7 = M1..M7; 8–255 reserved | Required 1..7 for MOVE (move or release); ignored otherwise |
| `direction` | int8 | −1, 0, +1 (operator sense) | MOVE: ±1 moves; **0 = release that channel** (level must be 0.0). Ignored for other actions |
| `level` | float32 | 0.0–1.0, fraction of channel `max_demand` | > 0 when moving, 0.0 on release; NaN/inf rejected; ignored for other actions |

### MotorCommand
| Field | Type | Unit / range | Notes |
|---|---|---|---|
| `channel` | uint8 | 1..7 | 0 invalid |
| `direction` | int8 | −1, 0, +1 (firmware convention) | polarity UNCALIBRATED |
| `demand` | float32 | 0.0–1.0 of firmware full scale | `0 ⇔ direction 0` |

### ArmCommand
| Field | Type | Unit / range | Notes |
|---|---|---|---|
| `header` | Header | ROS time | Creation time |
| `sequence` | uint32 | counter, wraps | New value per new command |
| `command_type` | uint8 | 0 INVALID, 1 SETPOINT, 2 STOP, 3 ESTOP, 4 REARM | |
| `motors` | MotorCommand[<=7] | exactly 7 required | See §2 |

### MotorState
| Field | Type | Unit | Valid when | Channels |
|---|---|---|---|---|
| `channel` | uint8 | 1..7 | always | all |
| `valid_flags` | uint8 | bits: 1 OUTPUT, 2 ENCODER, 4 ANGLE, 8 CURRENT | — | all |
| `applied_output` | float32 | −1..1 normalized, post-ramp | VALID_OUTPUT | all |
| `encoder_counts` | int32 | raw counts, **motor-side** | VALID_ENCODER | M1, M4, M5, M6 |
| `joint_angle_rad` | float32 | radians, **joint** (AS5600) | VALID_ANGLE | M2, M3 |
| `current_a` | float32 | amperes | VALID_CURRENT | M7 |
| `fault_flags` | uint16 | bits: 1 SENSOR_INVALID, 2 OVERCURRENT_STALL, 4 MAX_RUNTIME, 8 DRIVER | — | all |

Invalid or absent readings: float = NaN, int = 0, flag clear. Consumers ignore the value.
Encoder counts are not joint angles; conversion needs calibration that does not exist yet.
M7 current is not gripper position.

### ArmState
| Field | Type | Notes |
|---|---|---|
| `header` | Header | ROS time telemetry was received/produced; used for staleness |
| `simulated` | bool | true from the mock bridge; never shown as hardware |
| `firmware_state` | uint8 | 0 UNKNOWN, 1 BOOT, 2 IDLE, 3 ACTIVE, 4 CONTROLLED_STOP, 5 FAULT, 6 RECOVERY |
| `telemetry_sequence` | uint32 | gaps = lost frames |
| `accepted_sequence_valid` | bool | true only if the protocol explicitly confirms acceptance |
| `last_accepted_command_sequence` | uint32 | meaningful only if the flag above is true |
| `command_age_ms` | uint16 | firmware time since last valid command; 65535 = unknown |
| `active_faults` | uint32 | bits: 1 COMMAND_TIMEOUT, 2 ESTOP_LATCHED, 4 SENSOR_INVALID, 8 GRIPPER_OVERCURRENT, 16 GRIPPER_MAX_RUNTIME, 32 DRIVER, 64 FIRMWARE_WATCHDOG, 128 PROTOCOL_ERROR |
| `motors` | MotorState[<=7] | exactly 7 required before publishing |

### ArmDiagnostics
| Field | Type | Notes |
|---|---|---|
| `header` | Header | Report time |
| `simulated` | bool | mock bridge statistics are simulated |
| `link_state` | uint8 | 0 DISCONNECTED (default), 1 CONNECTING, 2 CONNECTED, 3 TELEMETRY_STALE, 4 PROTOCOL_MISMATCH |
| `local_protocol_version`, `remote_version_valid`, `remote_protocol_version` | uint8, bool, uint8 | Mismatch ⇒ no motion sent |
| `last_telemetry_age_s` | float32 | NaN if none yet |
| `commands_received`, `commands_rejected` | uint32 | ROS-side validation results |
| `frames_transmitted`, `frames_received_valid`, `frames_crc_error`, `frames_malformed` | uint32 | Transport counters |
| `telemetry_timeouts`, `reconnect_count` | uint32 | |
| `latched_faults` | uint32 | Same bits as `ArmState.FAULT_*`; cleared only by firmware after REARM |

## 5. Staleness and simulation

- `ArmState` is published only on new telemetry. `arm_state_monitor` (Phase 3) compares
  `header.stamp` with the current time and reports STALE past a configurable threshold;
  stale data is never displayed as current.
- `simulated = true` data must be labelled in logs and the operator display.

## 6. Reasoning for the design

- **Six messages, not more**: intent (`UserInput`), request (`ArmCommand` + `MotorCommand`),
  measurement (`ArmState` + `MotorState`), link health (`ArmDiagnostics`). Intent and
  request are separate so software limits and fault gating have a defined place.
- **Typed enums and bit flags, no strings** for control data.
- **Transport-neutral**: nothing refers to serial; a CAN transport can encode the same
  fields. Bounded arrays map cleanly to fixed-size frames.
- **Ramp parameters are deliberately absent**: the firmware owns ramping. If the final
  protocol exposes ramp configuration, it will be added then.

## 7. Tests proving the above

Implemented in `arm_control/test/test_command_validation.py` and `test_manual_control_core.py`
(plain Python, run here). They prove that none of these can produce motion: missing channel, duplicate channel, channel 0 or 8, six or eight entries, empty
array, INVALID type, NaN/inf demand, demand > 1 or < 0, direction ±2, direction 0 with
non-zero demand, non-zero demand with direction 0, SETPOINT while ESTOP is latched,
non-zero motors in STOP/ESTOP/REARM, REARM followed by no motion until a new SETPOINT,
stale `UserInput`, and bridge reconnect without operator REARM.
