"""Pure-Python bridge types shared by both bridges and msg_convert (no ROS imports)."""
import math
from dataclasses import dataclass

LINK_DISCONNECTED, LINK_CONNECTING, LINK_CONNECTED, LINK_TELEMETRY_STALE, LINK_PROTOCOL_MISMATCH = range(5)


@dataclass
class BridgeStats:
    """Everything /arm/diagnostics reports; owned by whichever bridge core is running."""
    simulated: bool = False
    link_state: int = LINK_DISCONNECTED
    remote_version_valid: bool = False
    remote_protocol_version: int = 0
    last_telemetry_age_s: float = math.nan
    commands_received: int = 0
    commands_rejected: int = 0
    frames_transmitted: int = 0
    frames_received_valid: int = 0
    frames_crc_error: int = 0
    frames_malformed: int = 0
    telemetry_timeouts: int = 0
    reconnect_count: int = 0
    latched_faults: int = 0
