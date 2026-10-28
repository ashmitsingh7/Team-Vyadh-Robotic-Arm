# MotorCommand: REQUESTED motion for ONE channel, embedded in ArmCommand.
#
# This is a request. The STM32 owns ramping: it moves the actual output toward this
# target gradually. The ROS side never sends ramp steps and never learns "motion occurred"
# from this message.

# Channel identifier, 1..7 = M1..M7 (same numbering as UserInput.CHANNEL_*).
# 0 (default) is invalid. Every channel M1..M7 must appear exactly once in an ArmCommand.
uint8 channel

# Requested direction in the FIRMWARE output convention (after ROS-side polarity mapping).
# What "positive" means mechanically is UNCALIBRATED until verified on hardware.
int8 DIRECTION_NEGATIVE=-1
int8 DIRECTION_STOP=0
int8 DIRECTION_POSITIVE=1
int8 direction

# Requested output magnitude, normalized fraction in [0.0, 1.0] of the firmware's
# full-scale demand for this channel (firmware applies its own per-channel limits on top).
# Consistency rule (enforced by validators): demand == 0.0 if and only if
# direction == DIRECTION_STOP. NaN/inf/out-of-range => the whole ArmCommand is rejected.
# Default (0.0, DIRECTION_STOP) = "ramp to zero".
float32 demand
