"""Mock bridge logic (pure Python): same command path as the real bridge, simulated firmware.

Commands are validated, gated (no automatic resume), then run through the REAL protocol
encoder and decoder before reaching the simulator, so the mock exercises the same
quantization and validation as the serial bridge. Everything it produces is SIMULATED.
"""
import math
from typing import List, Optional

from arm_control.command_validation import validate_arm_command
from arm_hardware.command_gate import COMMAND_TYPE_SETPOINT, MotionGate
from arm_hardware.firmware_sim import FirmwareSimModel
from arm_hardware.bridge_types import (LINK_CONNECTED, LINK_TELEMETRY_STALE, BridgeStats)
from arm_hardware.serial_protocol import (FrameDecoder, decode_command_frame, encode_command)
from arm_hardware.telemetry_parser import TelemetryData
from types import SimpleNamespace


class MockBridgeCore:
    def __init__(self, command_timeout_s: float = 0.5, telemetry_stale_after_s: float = 0.5):
        self.model = FirmwareSimModel(command_timeout_s=command_timeout_s)
        self.gate = MotionGate()
        self.stats = BridgeStats(simulated=True, remote_version_valid=True, remote_protocol_version=1)
        self.stale_after_s = telemetry_stale_after_s
        self.comm_loss = False                     # simulated cable pull / dead firmware
        self._last_telemetry: Optional[float] = None
        self._events: List[str] = []

    def drain_events(self) -> List[str]:
        ev, self._events = self._events, []
        return ev

    # ---- fault injection (tests / `ros2 param set`) --------------------------------
    def set_comm_loss(self, lost: bool, now: float) -> None:
        if lost != self.comm_loss:
            self.comm_loss = lost
            self._events.append(f'SIMULATED communication loss {"ON" if lost else "OFF"}')
            if not lost:
                self.gate.close()                  # recovery never auto-resumes motion

    def set_invalid_sensor_channel(self, channel: int) -> None:
        self.model.invalid_channels = {channel} if 1 <= channel <= 7 else set()

    # ---- commands -----------------------------------------------------------------
    def on_command(self, cmd, now: float) -> str:
        """Returns one of: 'delivered', 'rejected_invalid', 'gated_zero', 'lost'."""
        self.stats.commands_received += 1
        problems = validate_arm_command(cmd)
        if problems:
            self.stats.commands_rejected += 1
            self._events.append('rejected invalid ArmCommand: ' + '; '.join(problems))
            return 'rejected_invalid'
        healthy = self.link_state(now) == LINK_CONNECTED
        decision = self.gate.decide(cmd, healthy)
        outcome = 'delivered'
        if decision.substitute_zero:
            self.stats.commands_rejected += 1
            self._events.append(decision.reason)
            cmd = SimpleNamespace(command_type=COMMAND_TYPE_SETPOINT, sequence=cmd.sequence,
                                  motors=[SimpleNamespace(channel=c, direction=0, demand=0.0)
                                          for c in range(1, 8)])
            outcome = 'gated_zero'
        self.stats.frames_transmitted += 1
        if self.comm_loss:
            return 'lost'                           # transmitted into the void
        (frame,) = FrameDecoder().feed(encode_command(cmd))
        self.model.handle_command(decode_command_frame(frame), now)
        return outcome

    # ---- time / telemetry ---------------------------------------------------------
    def link_state(self, now: float) -> int:
        if self._last_telemetry is None:
            return LINK_CONNECTED if not self.comm_loss else LINK_TELEMETRY_STALE
        if self.comm_loss and now - self._last_telemetry > self.stale_after_s:
            return LINK_TELEMETRY_STALE
        return LINK_CONNECTED

    def tick(self, now: float) -> Optional[TelemetryData]:
        """Advance the simulator; returns telemetry to publish, or None during comm loss."""
        before = self.stats.link_state          # last PUBLISHED link state
        self.model.step(now)
        data = None
        if not self.comm_loss:
            data = self.model.telemetry(now)
            self._last_telemetry = now
            self.stats.frames_received_valid += 1
        after = self.link_state(now)
        if before == LINK_CONNECTED and after != LINK_CONNECTED:
            self.stats.telemetry_timeouts += 1
            self.gate.close()
        self.stats.link_state = after
        self.stats.last_telemetry_age_s = (math.nan if self._last_telemetry is None
                                           else max(0.0, now - self._last_telemetry))
        self.stats.latched_faults = 2 if self.model.estop_latched else 0   # ESTOP_LATCHED bit
        return data
