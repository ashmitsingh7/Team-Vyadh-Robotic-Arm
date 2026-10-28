# ArmDiagnostics: bridge/link health on /arm/diagnostics (published at a fixed low rate and
# on link-state changes). Separate from ArmState so link problems are visible even when no
# telemetry is arriving.
#
# Counters are since bridge start, wrap at 2^32. Counting a frame as "transmitted" means
# bytes were handed to the OS serial port; it says nothing about receipt or acceptance.

# Stamp = ROS time of this report. frame_id unused.
std_msgs/Header header

# True when published by mock_arm_bridge (all link statistics are simulated).
bool simulated

# Default (0) is DISCONNECTED, the conservative reading.
uint8 LINK_DISCONNECTED=0
uint8 LINK_CONNECTING=1
uint8 LINK_CONNECTED=2            # port open and recent valid telemetry
uint8 LINK_TELEMETRY_STALE=3      # port open but no valid telemetry within the timeout
uint8 LINK_PROTOCOL_MISMATCH=4    # firmware protocol version differs; motion must not be sent
uint8 link_state

# Protocol versions. remote_protocol_version is meaningful only if remote_version_valid.
uint8 local_protocol_version
bool remote_version_valid
uint8 remote_protocol_version

# Seconds since last valid telemetry; NaN if none received yet.
float32 last_telemetry_age_s

# ROS-side: ArmCommand messages received from /arm/command, and those the bridge rejected
# as invalid (never transmitted).
uint32 commands_received
uint32 commands_rejected

# Transport-side counters.
uint32 frames_transmitted          # command frames written to the port
uint32 frames_received_valid       # telemetry frames passing framing + CRC + range checks
uint32 frames_crc_error
uint32 frames_malformed            # bad length/version/type/field range, or framing lost
uint32 telemetry_timeouts          # times telemetry went stale
uint32 reconnect_count

# Fault bits LATCHED in the firmware as last reported; same bit meanings as the
# ArmState.FAULT_* constants. Latched faults require explicit operator REARM; the bridge
# never clears them and never resumes motion on reconnect.
uint32 latched_faults
