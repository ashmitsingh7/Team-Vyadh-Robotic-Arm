# ArmCommand: structured request to the hardware bridge on /arm/command.
#
# SEMANTICS: ArmCommand is always a FULL SEVEN-CHANNEL SNAPSHOT of the desired state,
# never a per-channel update. A channel that should not move is sent explicitly as
# (DIRECTION_STOP, 0.0). Nothing is ever "kept" from a previous command.
#
# Validity (all enforced by validators; any violation rejects the WHOLE command and it is
# counted in diagnostics, never partially applied or silently clamped):
#   * command_type != COMMAND_TYPE_INVALID
#   * `motors` has exactly NUM_CHANNELS entries, channels 1..7 each present exactly once
#     (any order), every entry individually valid (see MotorCommand)
#   * for REARM every entry must be (DIRECTION_STOP, 0.0)
# EXCEPTION (safety): STOP and ESTOP are honoured on command_type + sequence alone. Their
# `motors` content is ignored and never transmitted, so malformed channel data can neither
# block a stop nor cause motion. (arm_control always sends all-zero entries anyway.)
#
# A default-constructed message has command_type = INVALID and an empty `motors` list, so
# it can never cause motion.
#
# Publishers re-send the current snapshot at a fixed rate (heartbeat). The STM32 command
# timeout is the authoritative protection if publishing stops; the rate and timeout are
# configuration/firmware decisions, not defined here.

# Stamp = time the command was created by arm_control. frame_id unused.
std_msgs/Header header

# arm_control-local counter, wraps at 2^32. Increments for every new published command.
# Used to relate transmitted frames to firmware-reported accepted sequence (if the protocol
# confirms acceptance). Publishing a message does NOT mean it was executed.
uint32 sequence

uint8 COMMAND_TYPE_INVALID=0
uint8 COMMAND_TYPE_SETPOINT=1   # apply `motors` as the new full target snapshot
uint8 COMMAND_TYPE_STOP=2       # NORMAL stop: all channels ramp to zero by the firmware's controlled-stop policy
uint8 COMMAND_TYPE_ESTOP=3      # software emergency-stop REQUEST: firmware emergency response; latched until REARM
uint8 COMMAND_TYPE_REARM=4      # request to clear a latched ESTOP/fault; outputs stay zero; no motion implied
uint8 command_type

uint8 NUM_CHANNELS=7
MotorCommand[<=7] motors
