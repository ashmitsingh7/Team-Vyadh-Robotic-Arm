# ArmState: combined measured state on /arm/state, one message per decoded telemetry frame
# (real bridge) or per simulated update (mock bridge).
#
# It reports what the firmware SAID. It does not prove a command was executed or that the
# mechanism moved. "Transmitted", "received", "accepted" and "motion observed" are different
# facts; see docs/communication/ros_interface_spec.md.
#
# Staleness: this message is only published when new telemetry arrives. Consumers MUST judge
# freshness from header.stamp age (arm_state_monitor does this); they must not assume the
# latest message is current.

# Stamp = ROS time at which the bridge received/produced this telemetry. frame_id unused.
std_msgs/Header header

# True when the data is SIMULATED by mock_arm_bridge. Must never be presented as hardware.
bool simulated

# Firmware state machine state, as reported. UNKNOWN (0) is the default.
uint8 FW_STATE_UNKNOWN=0
uint8 FW_STATE_BOOT=1
uint8 FW_STATE_IDLE=2
uint8 FW_STATE_ACTIVE=3
uint8 FW_STATE_CONTROLLED_STOP=4
uint8 FW_STATE_FAULT=5
uint8 FW_STATE_RECOVERY=6
uint8 firmware_state

# Telemetry frame counter from the firmware (wraps at 2^32); gaps indicate lost frames.
uint32 telemetry_sequence

# ACCEPTANCE: set only if the serial protocol explicitly reports the sequence number of the
# last command the firmware ACCEPTED (passed validation). If false, the value below is
# meaningless and no acceptance may be claimed.
bool accepted_sequence_valid
uint32 last_accepted_command_sequence

# Firmware-measured time since its last VALID command, in milliseconds, saturating at 65534.
# 65535 = unknown / not reported.
uint16 COMMAND_AGE_UNKNOWN=65535
uint16 command_age_ms

# Firmware-level fault bits ACTIVE now. Meanings are a proposal; thresholds live in firmware.
uint32 FAULT_COMMAND_TIMEOUT=1       # command timeout / communication-loss response active
uint32 FAULT_ESTOP_LATCHED=2         # emergency stop latched; needs REARM
uint32 FAULT_SENSOR_INVALID=4        # at least one sensor invalid
uint32 FAULT_GRIPPER_OVERCURRENT=8   # M7 overcurrent/stall protection tripped
uint32 FAULT_GRIPPER_MAX_RUNTIME=16  # M7 maximum run-time exceeded
uint32 FAULT_DRIVER=32               # driver/power fault
uint32 FAULT_FIRMWARE_WATCHDOG=64    # firmware reset by its own watchdog
uint32 FAULT_PROTOCOL_ERROR=128      # firmware rejected frames (bad CRC/version/range)
uint32 active_faults

# Per-channel state. Valid telemetry has exactly 7 entries, channels 1..7 once each
# (checked by telemetry_parser / mock bridge before publishing).
uint8 NUM_CHANNELS=7
MotorState[<=7] motors
