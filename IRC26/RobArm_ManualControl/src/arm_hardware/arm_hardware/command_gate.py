"""Bridge-side motion gate (pure Python): no automatic resume after (re)connection.

Rule: a non-zero SETPOINT is only transmitted when the link is healthy AND a zero snapshot
(all-zero SETPOINT or STOP) has been seen since the link last became healthy. Otherwise the
bridge substitutes a zero SETPOINT. So a key still held during a cable re-plug, a bridge
restart or telemetry recovery cannot restart movement; the operator must let go (or STOP)
first. STOP/ESTOP always pass. REARM passes but does not open the gate.

This is an extra ROS-side layer; the STM32 must independently refuse to resume.
"""
from dataclasses import dataclass

COMMAND_TYPE_SETPOINT = 1
COMMAND_TYPE_STOP = 2
COMMAND_TYPE_ESTOP = 3
COMMAND_TYPE_REARM = 4


@dataclass(frozen=True)
class GateDecision:
    transmit_as_is: bool          # send the command unchanged
    substitute_zero: bool         # send an all-zero SETPOINT instead (command counted as rejected)
    reason: str = ''


def is_zero_snapshot(cmd) -> bool:
    return all(m.direction == 0 and m.demand == 0.0 for m in cmd.motors)


class MotionGate:
    def __init__(self):
        self.open = False

    def close(self) -> None:
        """Call whenever the link leaves the healthy state or (re)connects."""
        self.open = False

    def decide(self, cmd, link_healthy: bool) -> GateDecision:
        """`cmd` must already be validated."""
        ctype = cmd.command_type
        if ctype in (COMMAND_TYPE_STOP, COMMAND_TYPE_ESTOP):
            if ctype == COMMAND_TYPE_STOP and link_healthy:
                self.open = True
            return GateDecision(True, False)
        if ctype == COMMAND_TYPE_REARM:
            return GateDecision(True, False)
        # SETPOINT
        if is_zero_snapshot(cmd):
            if link_healthy:
                self.open = True
            return GateDecision(True, False)
        if not link_healthy:
            return GateDecision(False, True, 'link not healthy: non-zero setpoint replaced by zero')
        if not self.open:
            return GateDecision(False, True, 'no zero command seen since (re)connect: operator must '
                                             'release/STOP before motion can resume')
        return GateDecision(True, False)
