# Manual control (keyboard → ArmCommand)

Pipeline: `keyboard_teleop_node` → `/arm/user_input` → `manual_control_node` → `/arm/command`
→ bridge (mock or serial) → STM32. The ROS side **requests**; the STM32 ramps and protects.

## Keys (defaults, `arm_teleop/key_map.py`)

M1..M7 positive/negative: `q/a w/s e/d r/f t/g y/h u/j`. `SPACE` = STOP, `x` = ESTOP request,
`Z` = REARM, `+`/`-` = level, `?` = help, `Q` = quit. Input is a terminal (cbreak via termios),
so it works over SSH but needs a real TTY: run it in its own terminal, not from a launch file.
The input layer is replaceable (`key_source`), e.g. for a gamepad later.

## Hold window and stop latency

A terminal gives key presses, not releases. Each press/auto-repeat keeps a channel moving for
`hold_window_s` (default 0.6 s). When it expires the teleop sends an explicit release.
Worst-case stop latency after letting go of a key:

    hold window (0.6 s) + control tick + bridge period + firmware command path + firmware ramp-down

This is **not** the safety watchdog. The hardware command watchdog lives in the firmware.
For faster stops use SPACE (STOP) or `x` (ESTOP request).

## Distinct stop concepts

| Concept | Trigger | ROS side | Cleared by |
|---|---|---|---|
| Release | key hold-window expiry | channel target → 0 | next key press |
| STOP | SPACE | all zero + STOP repeated | next move |
| ESTOP request | `x` | latched, ESTOP published every cycle, motion refused | explicit REARM only |
| Comm loss | stale state / link down | motion blocked, targets dropped, bridge gates to zero | link recovers **and** operator releases/re-presses |
| Fault | blocking fault bits in ArmState | motion blocked | fault clears |

A software ESTOP is **not** a physical emergency stop.

## No automatic resume

REARM never implies motion (it is sent with an all-zero snapshot). After ESTOP, REARM,
link loss or reconnect, a key that is still held does **not** restart motion: the operator
must release it first (per-channel hold-off in the control core, plus the bridge MotionGate
that substitutes zeros until a zero snapshot has been seen).

## Gating and limits

Motion is blocked while ArmState is stale/missing, diagnostics are missing, firmware state is
not controllable, or a blocking fault bit is set (default mask 98 = ESTOP_LATCHED | DRIVER |
FIRMWARE_WATCHDOG; COMMAND_TIMEOUT is excluded on purpose so the arm cannot deadlock itself).
Joint limits for M2/M3 are off by default; polarity, max demand and limits in
`arm_bringup/config/channels.yaml` are **uncalibrated placeholders**.

## Validation

Any malformed ArmCommand is rejected whole. STOP/ESTOP are honoured on type + sequence alone.
Default-constructed messages are INVALID. See `docs/ros_interface_spec.md`.
