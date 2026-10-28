# STM32 NUCLEO-F446ZE firmware — requirements and implementation guide

**Status: no firmware exists in this repository.** This file is the checklist and design
brief for writing it. The ROS 2 side (everything else in this repo) has been written against
the contract below; none of it has ever run against real firmware or hardware. Where this
document says **TBD**, the value is a decision for the team to make from hardware data. It is
deliberately not guessed.

Read these first (they are the contract; this README does not repeat the byte layouts):

| Document | What it fixes |
|---|---|
| `docs/serial_protocol.md` | Framing (COBS + CRC-16), message types, payload layouts, units, test vectors |
| `docs/ros_interface_spec.md` | What the host sends/expects: STOP vs ESTOP vs REARM, acceptance semantics |
| `docs/Team_Vyadh_IRC_2027_Robotic_Arm_Data_Flow_Architecture.docx` | Overall system design and acceptance checklist |
| `docs/ramping_behavior.md` | The nine ramp/stop behaviours the firmware must implement |

The protocol is a **draft proposal**. The firmware developer should review it, raise
objections, and agree a frozen version before the first real flash. Change the document and
the host code together, never one side only.

---

## 1. The principle that drives everything

> **The STM32 is authoritative for physical outputs. The Jetson only requests.**

The Jetson, ROS 2, the USB cable and the operator's keyboard can all fail, hang, restart or
send garbage at any moment. The firmware must keep the arm safe **without relying on any of
them**. Concretely, the firmware must never assume that:

- a command arrived, is fresh, or is the last one the operator wanted;
- the host will send a stop before it disappears;
- the host's software limits, polarity mapping or "motion gate" worked (the host has them
  as an extra layer; they are not yours to rely on);
- the host will ever ramp. **The host sends a target only.** All ramping is yours.

A software ESTOP over serial is a *request*, not an emergency stop. A physical emergency
stop that removes motor power independently of the MCU is a separate hardware requirement
(§11) and the firmware must not be the only thing between a person and a moving arm.

## 2. Responsibility split (what you own)

| Firmware owns | Host (ROS 2) owns |
|---|---|
| Parsing/validating every incoming byte | Producing validated commands at a fixed rate |
| Command freshness watchdog and the communication-loss response | Operator input, key hold windows, ROS-side gating |
| Ramp generation for all 7 channels, reversal sequencing | Requesting direction + demand (no ramp steps) |
| Motor driver outputs, dead time, brake/coast policy | Per-channel polarity mapping *(host-side; see §6.5)* |
| ESTOP/fault response, latching, REARM conditions | Showing faults; never clearing them by itself |
| Sensor acquisition, calibration, plausibility, validity flags | Displaying telemetry, marking stale/simulated data |
| Gripper stall/overcurrent and max-run-time protection | — |
| Telemetry at a fixed rate | Detecting stale telemetry / link loss |

## 3. Hardware context and things to verify early

Target: NUCLEO-F446ZE (STM32F446ZE, Cortex-M4F, 180 MHz max).

Seven motor channels: **M1** turntable (incremental encoder), **M2** shoulder linear actuator
(AS5600 at the pivot), **M3** elbow linear actuator (AS5600 at the pivot), **M4** wrist pitch
(encoder), **M5/M6** wrist differential motors A/B (encoders, independent), **M7** gripper
lead screw (current sense only, **no position**). Drivers: the team's brushed-DC motor
controller PCB (see repo memory/notes) — confirm its input scheme (PWM + DIR, or two PWMs),
its dead-time behaviour, and its fault output before writing the output layer.

Verify these on the real board/schematic before committing to a design (flagged because they
are easy to get wrong, not because they are known to be wrong):

1. **USB serial path.** The ST-LINK virtual COM port is a real UART (commonly USART3 on
   PD8/PD9 on Nucleo-144 boards — check your board revision's user manual and solder bridges).
   Baud rate therefore matters and must match the host (`baudrate`, default 115200). A native
   USB OTG CDC port would ignore baud but needs a USB stack.
2. **Two AS5600 sensors share one fixed I²C address (0x36).** M2 and M3 therefore need either
   two independent I²C buses, an I²C multiplexer, or sensors with a configurable address.
   Decide this early; it shapes the sensor layer. Also confirm which interface you read
   (I²C angle registers vs PWM/analog output).
3. **Timer allocation.** Four encoders need four timers with encoder mode (TIM1/2/3/4/5/8 on
   this chip; TIM2 and TIM5 are 32-bit, the others 16-bit). Seven motor PWM outputs plus a
   ramp-tick timer need further timers/channels. Draw the resource map before pin assignment.
4. **Gripper current sensor** scaling (ADC counts → amperes), reference voltage, shunt/amplifier
   gain, noise, and the motor's start-up inrush (§9.3).
5. **Physical E-stop** circuit and what it disconnects (§11).
6. **Power-up pin states.** Motor-driver inputs must have defined pull-up/pull-down so the
   outputs are inert *before* your firmware starts and while the MCU is in reset.

## 4. Host contract at a glance (see the protocol doc for bytes)

- Link: 8N1, default 115200, no flow control. Frames are `COBS(version|type|len|payload|crc16) + 0x00`,
  little-endian, CRC-16/CCITT-FALSE (check value of `123456789` is `0x29B1`).
- Host → firmware: SETPOINT (7 × int16 signed per-mille, −1000..1000), STOP, ESTOP, REARM, each
  with a `u32 command_seq`. STOP/ESTOP/REARM carry no motor data by design.
- Firmware → host: TELEMETRY (139-byte payload) at a fixed rate (default 20 Hz), always, even
  with no commands.
- The host re-sends a SETPOINT at a fixed rate (default 20 Hz) as a heartbeat. A *valid* command
  of any type refreshes your watchdog.
- **Nothing in this repo has been checked against firmware.** Use the byte-exact vectors in
  `serial_protocol.md` §8 (the host's own tests verify them) as your conformance tests.

## 5. Firmware architecture (recommended)

Keep protocol logic, ramping and the safety state machine in **plain C modules with no HAL
calls**, so they can be unit-tested on a PC with the same vectors. Wrap hardware in thin
drivers. Suggested layout:

```
firmware/
  Core/Inc, Core/Src          CubeMX-generated init + your app glue
  App/protocol/               cobs.c, crc16.c, frame_parser.c, command_codec.c, telemetry_codec.c   (pure C)
  App/control/                state_machine.c, ramp.c, safety.c, watchdog_cmd.c                      (pure C)
  App/drivers/                motor_out.c, encoder.c, as5600.c, current_sense.c, estop_in.c, uart_dma.c (HAL)
  App/config/                 vyadh_config.h   <- every TBD/calibration constant in ONE place
  Tests/                      host-side unit tests + vector tests, and HIL test notes
```

Rules for the whole code base:

- **No dynamic allocation** (`malloc`/`new`). Static, bounded buffers everywhere.
- **No blocking waits** in the control path (no `HAL_Delay`, no polling a peripheral for
  completion in the main/control loop, no unbounded loops on I²C/UART flags). Every external
  wait has a timeout and a defined failure result.
- **ISRs only move data**: UART/DMA ISR copies bytes into a ring buffer; the parser runs in
  the main loop or a low-priority task. Never parse, ramp, or do I²C inside a UART ISR.
- **Interrupt priorities:** the ramp/safety tick and any hardware fault inputs must be able to
  pre-empt communication handling. A flooded UART must never delay the safety tick.
- **Independent hardware watchdog (IWDG)** running, fed only from the main supervisory loop
  once all periodic tasks have reported in (not from a timer ISR that keeps running while the
  main loop is hung). Report a watchdog reset in telemetry (`FIRMWARE_WATCHDOG`).
- Use fixed-width integers; define units in names (`_permille`, `_urad`, `_ma`).
- Make every threshold a named constant in `vyadh_config.h` with a comment stating its unit,
  its source (datasheet/measurement) and whether it is calibrated.

## 6. Command reception and validation

### 6.1 Receiving bytes
- UART with **DMA circular reception + IDLE-line interrupt** (or equivalent) into a ring
  buffer sized for at least several frames (frames are ≤ 207 bytes encoded). Handle UART
  errors (overrun, framing, noise) by clearing flags and counting them; never let an error
  state stall reception.
- Bound everything: if no `0x00` delimiter arrives within the maximum encoded frame size
  (207 bytes), discard bytes until the next delimiter and count an overflow.

### 6.2 Parsing (all steps must pass or the frame is discarded)
1. Split on `0x00`; ignore empty chunks.
2. COBS-decode; reject on any structural error.
3. Body length ≥ 5 and `len` consistent with body size; `len ≤ 200`.
4. **CRC-16 over `version|type|len|payload`** matches.
5. `version == 1` (the host currently sends 1). On another version: do not act on SETPOINTs;
   count it; (STOP/ESTOP handling on a mismatched version is your safety decision — document it).
6. Type known; payload length exactly as specified for that type; SETPOINT values within ±1000.
7. Only now does the command take effect.

A frame that fails **any** check **must not change any motor target** and **must not refresh the
command watchdog**. Count failures by category (CRC, framing/COBS, length, range, version) and
report them (`PROTOCOL_ERROR` fault bit / counters as you define).

### 6.3 Sequence numbers and acceptance
- `command_seq` (u32) is host-assigned. After a command **passes validation and is applied**,
  set `last_accepted_cmd_seq` and `accepted_seq_valid = 1` in telemetry. This is the *only*
  acceptance signal the host recognises. Do not report acceptance for rejected frames
  (e.g. SETPOINT while ESTOP is latched). Before any command is accepted, send
  `accepted_seq_valid = 0`.
- The host does not rely on sequence monotonicity for safety, but you may drop duplicates/very old sequences if you define a freshness rule (document it; TBD).

### 6.4 Priority
- **ESTOP is processed with priority over everything**: handle it as soon as it is parsed,
  ahead of any queued SETPOINTs, and never defer it behind slow work.
- A parse-valid STOP/ESTOP must act even if the rest of the system state is odd.

### 6.5 What the host does *not* do for you
Per-channel **polarity** is applied by the host (config placeholder, uncalibrated), so on the
wire `+` means "firmware positive". You still define what "positive" physically means for each
driver. Document each channel's convention in `vyadh_config.h` once measured.

## 7. Safe boot, reset and state machine

### 7.1 Boot
- Outputs inert before `main()` (hardware pull states) and set to the safe state first thing
  in init, **before** peripherals that could block (I²C, USB, clocks) are brought up.
- State `BOOT` → `IDLE` only after peripherals initialise and self-checks pass. **No motor
  moves during or after boot** until a valid *non-zero* SETPOINT is received (after any
  REARM requirement). A reset (power, brown-out, IWDG, host reboot, cable re-plug) always
  returns to outputs-zero with no remembered target.
- Report the reset cause (`FIRMWARE_WATCHDOG` for IWDG, plus whatever you add for BOR/NRST).

### 7.2 States (reported in telemetry `firmware_state`)

| State | Value | Meaning | Outputs |
|---|---|---|---|
| UNKNOWN | 0 | Never send normally; for host "no information" | — |
| BOOT | 1 | Initialising | Zero |
| IDLE | 2 | Ready, no motion requested | Zero |
| ACTIVE | 3 | Fresh valid commands, ramp controlling outputs | Ramped toward target |
| CONTROLLED_STOP | 4 | Normal STOP or communication-loss stop in progress | Ramping to zero by stop policy |
| FAULT | 5 | A serious fault or ESTOP latched | Per fault/ESTOP policy; stays until cleared |
| RECOVERY | 6 | REARM accepted, re-checking | Zero |

Suggested transitions:

| From | Event | To |
|---|---|---|
| BOOT | init done, checks pass | IDLE |
| IDLE/ACTIVE | valid non-zero SETPOINT | ACTIVE |
| ACTIVE | all targets zero and outputs at zero | IDLE |
| ACTIVE/IDLE | STOP | CONTROLLED_STOP → IDLE when outputs reach zero |
| any | command timeout | CONTROLLED_STOP (comm-loss policy), `COMMAND_TIMEOUT` active |
| any | ESTOP | FAULT (ESTOP_LATCHED, latched) |
| any | serious fault | FAULT |
| FAULT | accepted REARM and conditions clear | RECOVERY → IDLE |
| FAULT | SETPOINT | stays FAULT; **rejected** (not accepted) |

REARM **never** starts motion. After RECOVERY→IDLE, outputs stay zero until a *new* non-zero
SETPOINT. Faults that require manual reset vs. self-clearing: TBD (decide per fault, §10).

## 8. Command watchdog (communication-loss protection) — owned by you

- Independent timer (not tied to UART interrupts firing). Tracks time since the last
  **valid** command. **Timeout value: TBD**; choose from motion risk and measured link timing.
  The host heartbeat is 20 Hz (50 ms), so the timeout must exceed several periods plus jitter
  (the host's own suggestion is ≥ 3 periods; a short timeout is safer but must not nuisance-trip).
- On timeout: apply the configured **communication-loss stop policy** locally (a controlled
  ramp-down, or faster if risk demands — TBD), set `COMMAND_TIMEOUT` active, keep telemetry
  flowing. The host cannot recover the arm; you must already have stopped it.
- After timeout, motion **must not resume** just because commands return: require a fresh
  non-zero SETPOINT *and* decide explicitly whether a zero command first is required (the
  host bridge sends a zero before motion after every reconnect, but you must not depend on it).
  Clear `COMMAND_TIMEOUT` when valid commands flow again.
- Report `command_age_ms` (saturating at 65534; 65535 = unknown) in every telemetry frame.
- This is *the* protection when the ROS node dies, the Jetson freezes or the cable is pulled.
  Test it with the cable physically removed (§14).

## 9. Ramp generator, motor outputs and per-channel protections

The ramp generator is a **fixed-rate periodic task** (timer-driven; rate TBD — typically
100 Hz to 1 kHz; choose from driver/motor response). Keyboard rate, ROS timing and UART
timing must never determine the ramp slope.

### 9.1 Per-channel state
At minimum: requested target (signed), current ramped output (signed), current/requested
direction, reversal phase (`RAMP_DOWN`, `DEAD_TIME`, `RAMP_UP`), and per-channel fault flags.

### 9.2 Behaviours you must implement
(See `docs/ramping_behavior.md` for the full table; every row needs a test.)

1. **Start from rest:** begin at zero output and increase gradually toward the target.
2. **Demand up/down:** move toward the new target at the configured rate; **never jump** to a
   new demand because a new command arrived. Rates may be asymmetric (accel vs decel).
3. **Normal stop / key release (target → 0 or STOP):** controlled deceleration to zero by the
   agreed stop policy (coast vs brake vs active ramp: TBD).
4. **Direction reversal:** ramp to zero → wait the safe reversal/dead-time interval → change
   direction → ramp up. Never flip direction at non-zero output. On H-bridges enforce
   **dead time** against shoot-through.
5. **Emergency stop / serious fault:** a *separately defined* response (TBD: coast, brake, or
   disable the driver). It must not use a long comfort ramp if that prolongs a hazard.
6. **Stale command / lost link:** local safe-stop policy (§8), not dependent on the host.
7. **Wrist differential (M5, M6):** the host addresses them independently and has **no**
   roll/yaw mapping. Your ramp must still apply *compatible* ramp constraints across the pair so
   an unintended abrupt mismatch cannot strain the differential. Define how (shared slew limit,
   proportional scaling) from the real gear geometry — TBD, do not guess signs.
8. **Gripper closure (M7):** ramp output while current-based protection independently watches
   for stall/contact and a **maximum run-time**; either stops M7.
9. **Output limits:** apply channel-specific maximum output (the host's `max_demand` is only a
   reduction on top of yours; you must enforce your own absolute maximum even if the host
   sends ±1000).

Write the ramp as integer/fixed-point math; define units (e.g. per-mille per tick) and state
the ramp rate in physical terms (e.g. "full scale in X ms") in comments. Ramping must be a
*different function from* closed-loop position control; none is needed in this milestone.

### 9.3 Gripper (M7) protection details
- Sample the current sensor with ADC+DMA, oversample/filter, convert with the real circuit
  calibration. **Do not invent thresholds**: measure free-running, loaded and stalled current.
- Blank the start-up inrush window so normal starts do not trip.
- Overcurrent/stall: stop (or limit) M7 per the validated threshold and duration; report
  `GRIPPER_OVERCURRENT` (global bit 0x08) and per-channel `OVERCURRENT_STALL` (0x02); decide latch/clear.
- Maximum run time per direction: stop M7 when exceeded even if no current trip; report
  `GRIPPER_MAX_RUNTIME` (0x10 / channel 0x04).
- Current ≠ position. Never report a gripper position derived from current.

## 10. Safety manager and fault handling

One module owns "what is the arm allowed to do right now". It overrides the ramp generator.

| Condition | Detect | Response | Latch / clear | Reported as |
|---|---|---|---|---|
| No valid command within timeout | watchdog | comm-loss stop (§8) | self-clears when valid commands return | `COMMAND_TIMEOUT` 0x01 |
| Malformed/corrupt/CRC-bad frame | parser | discard; do not touch target or watchdog | n/a | counters, `PROTOCOL_ERROR` 0x80 |
| ESTOP command | parser | emergency policy (TBD) | **latched**; needs accepted REARM | `ESTOP_LATCHED` 0x02, latched too |
| Physical E-stop input (if wired to MCU) | GPIO | same as ESTOP | per design | `ESTOP_LATCHED` |
| Encoder/AS5600 invalid or implausible | sensor layer | flag invalid; block features needing feedback; defined safe response | TBD | `SENSOR_INVALID` 0x04, ch bit 0x01 |
| Gripper overcurrent/stall | current monitor | stop/limit M7 | TBD | 0x08 / ch 0x02 |
| Gripper max run time | M7 timer | stop M7 | TBD | 0x10 / ch 0x04 |
| Driver fault/power issue | driver FLT pin, supply monitor | force safe outputs where detectable | TBD | `DRIVER` 0x20 / ch 0x08 |
| Firmware hang/reset | IWDG | reset to safe state | report next boot | `FIRMWARE_WATCHDOG` 0x40 |
| Unsupported protocol version | parser | ignore SETPOINTs | — | counters |

Decide and document (do not leave implicit): which faults latch, which clear themselves, what
exactly REARM verifies, and what the ESTOP output policy is. Telemetry must carry both
`active_faults` (now) and `latched_faults` (stay until cleared by an accepted REARM).

## 11. Hardware safety (outside firmware, but design the firmware to cooperate)

- A **physical emergency stop that removes motor power independent of the MCU and the
  Jetson** is required for any test with people near the arm. Software ESTOP is not a
  substitute. Consider reading the E-stop state into the MCU purely for reporting.
- Fusing, current limiting, mechanical restraints/stops, and safe test procedure are the
  team's responsibility. Do not commission with the mechanism suspended or anyone in the
  travel envelope until stop behaviour, limits, wiring, current thresholds and the
  communication timeout have all been tested.

## 12. Sensors

### 12.1 Incremental encoders (M1, M4, M5, M6)
Timer in encoder mode with input filtering. Handle 16-bit wrap (extend to 32-bit in software
at a rate faster than the maximum count rate) or use TIM2/TIM5. Report `encoder_counts` (i32)
as **raw motor-side counts**. Conversion to a joint angle needs counts-per-rev, gear ratio and
direction — all uncalibrated and **not** done on the host. Set `VALID_ENCODER` only when the
reading is trustworthy; send `0` when not.

### 12.2 AS5600 joint angles (M2, M3)
- Read the 12-bit angle over I²C with timeouts and bus-recovery (clock out stuck-bus conditions).
  Check the status register magnet flags (detected / too weak / too strong) — a missing or
  badly positioned magnet must clear `VALID_ANGLE`.
- Unwrap across the 0/4095 boundary, apply the zero offset and direction calibration
  (TBD — define zero angle, positive direction, plausibility bounds), and report
  `joint_angle_urad` (i32, micro-radians) of the **joint**, not the motor.
- Plausibility checks: rate-of-change bounds, stuck-value detection, communication errors.
  An invalid reading must never be sent as a plausible 0.

### 12.3 Validity discipline
Every reading has a valid bit (`VALID_OUTPUT`=1, `VALID_ENCODER`=2, `VALID_ANGLE`=4,
`VALID_CURRENT`=8). Fields whose bit is clear **must be sent as 0** and are ignored by the host
(it presents them as NaN/absent). Do not set bits for sensors that a channel does not have
(M1/M4/M5/M6 → encoder; M2/M3 → angle; M7 → current; all → output).

## 13. Telemetry

- Fixed rate (default 20 Hz; **TBD**), independent of commands, transmitted with **DMA/interrupt TX and
  a bounded queue**; a stalled host must never block the control loop. If TX can't keep up, drop
  telemetry frames, not safety work.
- `telemetry_seq` increments every frame (gaps show lost frames). Fill header and 7 channel
  records exactly as in the protocol doc; `applied_output_permille` is the **post-ramp output you
  are actually applying**, not the request.
- Bandwidth estimate: 146 bytes/frame ≈ 2.9 kB/s ≈ 25 % of a 115200 baud link at 20 Hz.

## 14. Testing and exit criteria

### 14.1 Host-side unit tests (no hardware)
Compile the pure-C protocol/control modules on a PC and test:
- the CRC check value, the COBS vectors, and **all command and telemetry vectors in `serial_protocol.md` §8**;
- parser robustness: partial frames, byte-by-byte delivery, every single-bit corruption rejected,
  garbage/overflow recovery, truncated frames, wrong length/type/version, out-of-range permille;
- ramp: start from rest, demand up/down, stop, reversal passing through zero + dead time, ESTOP
  response, no jump on new commands;
- state machine: every transition in §7, ESTOP latch, REARM clears nothing it shouldn't, no
  motion after REARM, no resume after timeout.

### 14.2 Bench tests with motor power **disabled** (logic analyser/scope on outputs)
Link and telemetry exchange; watchdog with the **USB cable removed**; malformed-frame flood
does not move outputs or starve the safety tick; reset behaviour (power cycle, reset button,
IWDG) always ends with outputs inert; no output glitch at boot or when the host connects.

### 14.3 Powered commissioning (one channel at a time, mechanism secured)
Confirm direction polarity, ramp up/down, reversal, stop, ESTOP and REARM, the timeout under
load; then remaining channels individually; then M5/M6 coordination; then gripper current/time
protection against calibrated thresholds. Record every measured value into `vyadh_config.h`
and update the ROS-side `arm_bringup/config/channels.yaml` polarity/limits accordingly.

### 14.4 Acceptance checklist (from the architecture doc — all must be demonstrably true)
- [ ] All seven channels have a unique, documented ID and verified direction mapping.
- [ ] A command cannot bypass the ramp generator.
- [ ] Normal demand changes ramp rather than producing abrupt output steps.
- [ ] Direction reversal passes through the agreed zero-output transition.
- [ ] Normal stop and emergency/fault stop are distinct and tested.
- [ ] Command timeout is tested with the Jetson disconnected / cable removed.
- [ ] Telemetry reports sensor validity and faults, not just numbers.
- [ ] Gripper stall protection uses calibrated current and a maximum run time.
- [ ] Framing, versioning, units and test vectors are documented and pass.
- [ ] Outputs start in a safe state after boot, reset and fault recovery.
- [ ] No automatic motion resumes after reconnect unless explicitly specified and tested.

## 15. Open decisions (must be made by people, from data — not guessed)

Baud rate and command/telemetry rates · command timeout value and comm-loss stop policy ·
ramp rates (accel/decel), reversal dead time, PWM frequency, ramp tick rate · ESTOP output
behaviour (coast/brake/disable) · which faults latch and how they clear · REARM conditions ·
per-channel absolute output limits and driver polarity · encoder CPR/gear ratios · AS5600
zero offsets, direction, wrap handling, plausibility bounds, and the shared-address solution ·
M5/M6 coordination rule (needs the differential geometry) · gripper current threshold, filter,
blanking and max run time · sequence freshness rule (if any) · behaviour on protocol version
mismatch · CAN migration path (keep the protocol layer separate from the transport driver).

## 16. Things that will hurt later if skipped

- Don't let the host's heartbeat or any single frame be your only notion of time — use a
  hardware timer for the watchdog and ramp.
- Don't parse in an ISR; don't `printf` over the same UART used for the protocol.
- Don't leave any output pin floating during boot or reset.
- Don't report a command as accepted before it has actually been applied.
- Don't clear a latched fault because "the cable came back".
- Don't add fields or bits to the protocol without bumping the version and updating the host
  code, its tests and the shared vectors at the same time.
