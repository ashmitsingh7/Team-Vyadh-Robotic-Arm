# UserInput: high-level operator INTENT. Published by arm_teleop on /arm/user_input.
#
# This is NOT a motor command. arm_control turns intent into an ArmCommand after
# applying software limits and fault/stale-state checks. See
# docs/communication/ros_interface_spec.md.
#
# Safe default: an all-zero (default-constructed) message has action = ACTION_INVALID
# and MUST be ignored/rejected by consumers.

# Time the input was generated (teleop node clock). frame_id unused.
std_msgs/Header header

# Teleop-local counter, wraps at 2^32. For logging/duplicate detection only.
uint32 sequence

# What the operator wants.
uint8 ACTION_INVALID=0   # default / unset: reject
uint8 ACTION_MOVE=1      # move `channel` in `direction` at `level`
uint8 ACTION_STOP=2      # normal (controlled) stop of all channels
uint8 ACTION_ESTOP=3     # software emergency-stop REQUEST (not a physical E-stop)
uint8 ACTION_REARM=4     # explicit operator re-arm after ESTOP/fault; never implies motion
uint8 action

# Target channel for ACTION_MOVE (move or release). 1..7 = M1..M7 (human numbering, see CHANNEL_* below).
# 0 = not applicable. For STOP/ESTOP/REARM every field except `action` is IGNORED, so a
# malformed field can never prevent a stop. 8..255 are reserved (e.g. future
# groups defined in configuration) and are rejected until a group scheme is documented.
uint8 CHANNEL_NONE=0
uint8 CHANNEL_M1=1       # turntable
uint8 CHANNEL_M2=2       # shoulder actuator
uint8 CHANNEL_M3=3       # elbow actuator
uint8 CHANNEL_M4=4       # wrist pitch
uint8 CHANNEL_M5=5       # wrist differential motor A (independent; no roll/yaw mapping)
uint8 CHANNEL_M6=6       # wrist differential motor B (independent; no roll/yaw mapping)
uint8 CHANNEL_M7=7       # gripper lead-screw motor
uint8 channel

# Operator-sense direction: -1 negative, +1 positive. For ACTION_MOVE, direction 0 (with
# level 0.0) means RELEASE this channel: its target returns to zero (ramped by the firmware)
# without affecting other channels. Ignored for non-MOVE actions.
# "Positive" here is the OPERATOR's sense; arm_control applies the configured per-channel
# polarity (UNCALIBRATED placeholder) to get the firmware convention used in ArmCommand.
int8 direction

# Requested speed level for ACTION_MOVE as a fraction in [0.0, 1.0] of the channel's
# configured ROS-side maximum demand (arm_bringup/config/channels.yaml). Not a PWM value.
# Ignored for non-MOVE actions. Must be 0.0 for a MOVE release (direction 0) and > 0.0 when
# direction is non-zero. NaN/inf/out-of-range => reject.
float32 level
