# Ramping and stop behaviour

**The STM32 owns ramping.** The Jetson/ROS side only sends a requested direction and demand
(`ArmCommand` → SETPOINT) and explicit STOP / ESTOP / REARM requests. It never sends ramp
steps, never treats keyboard repeat rate as a ramp, and exposes no ramp parameters. This repo
contains **no ramp generator** (a test, `test_no_ramping_code_on_the_ros_side`, guards that).

**Nothing about ramping has been physically tested.** The mock bridge and the fake STM32 do
*not* model ramping (their applied output jumps to the target); any ramp-like result from a
simulation must not be read as evidence that real ramping works.

The table states the intended end-to-end behaviour. "ROS side" is implemented and unit-tested
(plain Python / fake ROS); "Firmware" is the requirement in `firmware/README.md`,
**not yet implemented**.

| # | Situation | ROS side does | Firmware must do |
|---|---|---|---|
| 1 | Start from rest | Publishes SETPOINT with direction and demand = level × channel `max_demand` (placeholder 0.20) | Begin at zero output and increase gradually toward the target; never jump |
| 2 | Increase requested demand | Sends the new target in the next heartbeat (no intermediate steps) | Move output toward the new target at the configured rate |
| 3 | Decrease requested demand | Same, lower target | Decelerate toward the new target at the configured (possibly different) rate |
| 4 | Normal stop (key release → `MOVE` release, or STOP key) | Release: that channel's target goes to 0 in the next snapshot. STOP: all zero and `STOP` command repeated `stop_repeat` times | Controlled deceleration to zero by the agreed stop policy (coast/brake/active ramp: TBD) |
| 5 | Direction reversal | Sends the opposite-direction target; never inserts a zero itself | Ramp to zero, wait safe reversal/dead time, change direction, ramp up. Never flip direction at non-zero output |
| 6 | Emergency stop | Latches ESTOP; publishes `ESTOP` every cycle; refuses all movement until an explicit operator REARM. **A software request only — not a physical E-stop** | Separate emergency response (coast/brake/disable: TBD), latched until an accepted REARM; no long comfort ramp |
| 7 | Communication loss (cable pulled, Jetson/ROS dies, telemetry gone) | Marks telemetry STALE, blocks new motion, drops active targets; bridge replaces non-zero setpoints by zero and never resumes automatically after reconnect (needs release / new key press). These are *extra* layers | **Command timeout** (value TBD) triggers a local safe-stop policy without waiting for the Jetson; reports `COMMAND_TIMEOUT`; does not resume on its own |
| 8 | Coordinated M5/M6 movement | Addresses M5 and M6 independently; **no roll/yaw mapping exists** (needs differential geometry) | Apply compatible ramp constraints to M5 and M6 so the differential sees no unintended abrupt mismatch (rule TBD) |
| 9 | Gripper closure (M7) | Sends M7 direction/demand; shows current/fault; does **not** infer position from current | Ramp output while current-based stall/contact protection and a maximum run time independently stop M7 (thresholds TBD, calibrated) |

## What the host guarantees (so firmware can rely on less, but must not rely on it)

- SETPOINT heartbeat at a fixed rate (default 20 Hz) while the link is up.
- Non-zero setpoints are only sent after a zero setpoint following every (re)connect, stale
  period or protocol mismatch.
- STOP and ESTOP are always transmitted, even with a stale link, and are honoured even if
  other fields in the request are malformed.
- Direction reversal is not pre-sequenced by the host; reversal handling is firmware-only.

## Not defined yet (TBD, must be decided from data)

Ramp tick rate, accel/decel rates, reversal dead time, PWM frequency, stop policy
(coast/brake/active), ESTOP output behaviour, command timeout, M5/M6 coordination rule,
gripper thresholds and run-time limit. They are intentionally absent from the ROS configuration.
