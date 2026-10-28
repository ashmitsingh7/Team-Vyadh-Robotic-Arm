# MotorState: MEASURED/REPORTED state of ONE channel, embedded in ArmState.
# Describes what the firmware reports, never what was requested.
#
# Validity rule: a value is meaningful ONLY if its VALID_* bit is set in `valid_flags`.
# When a bit is clear (sensor absent on this channel, or reading invalid) the matching
# field is unspecified and MUST be ignored; producers set floats to NaN and ints to 0 so
# an invalid reading is never mistaken for a plausible zero.

# Channel identifier, 1..7 = M1..M7. 0 (default) is invalid.
uint8 channel

uint8 VALID_OUTPUT=1
uint8 VALID_ENCODER=2
uint8 VALID_ANGLE=4
uint8 VALID_CURRENT=8
uint8 valid_flags

# Output currently applied by the firmware ramp generator (AFTER ramping), signed,
# normalized to [-1.0, 1.0] in the firmware output convention. Not the requested demand.
float32 applied_output

# Raw incremental-encoder counts, MOTOR-SIDE, signed, since firmware boot or last firmware
# zeroing. NOT a joint angle: conversion needs counts-per-rev, gear ratio and polarity
# (uncalibrated / TBD). Channels M1, M4, M5, M6 only.
int32 encoder_counts

# AS5600 JOINT angle in radians, after firmware zero offset/direction calibration
# (calibration TBD). Channels M2, M3 only. Not motor angle.
float32 joint_angle_rad

# Motor current in amperes (circuit calibration TBD). Channel M7 only. Indicates load/stall
# contact; it is NOT a gripper position.
float32 current_a

# Per-channel fault bits reported by the firmware. Bit meanings are a proposal; thresholds
# are defined in firmware, never invented on the ROS side.
uint16 FAULT_SENSOR_INVALID=1       # feedback missing/implausible
uint16 FAULT_OVERCURRENT_STALL=2    # current-based stall/contact protection tripped (M7)
uint16 FAULT_MAX_RUNTIME=4          # maximum run-time limit reached (M7)
uint16 FAULT_DRIVER=8               # driver fault reported/detected
uint16 fault_flags
