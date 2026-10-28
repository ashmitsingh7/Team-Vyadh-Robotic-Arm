"""Behavioural stand-in for the STM32 firmware, used by the mock bridge and test tools.

*** THIS IS NOT FIRMWARE AND NOT A MODEL OF THE REAL ARM. ***
It exists only so the ROS pipeline can be exercised without motors. Its outputs are
SIMULATED and must be labelled as such wherever they are shown. In particular it does NOT
model ramping: `applied_output` jumps to the target on the next step. Ramp generation,
the command watchdog and all real safety logic belong to the STM32 firmware; see
firmware/README.md for the requirements.

It models just enough of the documented firmware state machine (docs/
serial_protocol.md section 5) to test the ROS side: command acceptance with echoed
sequence numbers, STOP, latched ESTOP, REARM, command timeout, invalid-sensor reporting.
"""
import math
from typing import Dict, Optional, Set

from arm_hardware.serial_protocol import (NUM_CHANNELS, TYPE_ESTOP, TYPE_REARM, TYPE_SETPOINT,
                                          TYPE_STOP, DecodedCommand)
from arm_hardware.telemetry_parser import (COMMAND_AGE_UNKNOWN, VALID_ANGLE, VALID_CURRENT,
                                           VALID_ENCODER, VALID_OUTPUT, ChannelTelemetry,
                                           TelemetryData)

FW_BOOT, FW_IDLE, FW_ACTIVE, FW_CONTROLLED_STOP, FW_FAULT, FW_RECOVERY = 1, 2, 3, 4, 5, 6
FAULT_COMMAND_TIMEOUT, FAULT_ESTOP_LATCHED, FAULT_SENSOR_INVALID = 1, 2, 4

ENCODER_CHANNELS = (1, 4, 5, 6)
ANGLE_CHANNELS = (2, 3)
CURRENT_CHANNELS = (7,)


class FirmwareSimModel:
    def __init__(self, command_timeout_s: float = 0.5, encoder_counts_per_s: float = 2000.0,
                 angle_rad_per_s: float = 0.5, gripper_current_a: float = 1.0):
        self.command_timeout_s = command_timeout_s
        self._enc_rate = encoder_counts_per_s
        self._ang_rate = angle_rad_per_s
        self._cur_full = gripper_current_a
        self.fw_state = FW_BOOT
        self.targets = [0.0] * NUM_CHANNELS      # signed, [-1, 1]
        self.applied = [0.0] * NUM_CHANNELS      # equals targets after step(): NO ramp modelled
        self.encoder = [0] * NUM_CHANNELS
        self.angle = [0.0] * NUM_CHANNELS
        self.estop_latched = False
        self.accepted_valid = False
        self.last_accepted_seq = 0
        self.telemetry_seq = 0
        self.invalid_channels: Set[int] = set()
        self.rejected_commands = 0
        self._boot_time: Optional[float] = None
        self._last_cmd_time: Optional[float] = None
        self._last_step: Optional[float] = None
        self._timeout_active = False

    # ---- inputs -------------------------------------------------------------------
    def handle_command(self, cmd: DecodedCommand, now: float) -> bool:
        """Apply a parse-valid command. Returns True if ACCEPTED (applied), False if rejected."""
        self._last_cmd_time = now        # any parse-valid frame proves the link is alive
        self._timeout_active = False
        if cmd.msg_type == TYPE_ESTOP:
            self.targets = [0.0] * NUM_CHANNELS
            self.applied = [0.0] * NUM_CHANNELS
            self.estop_latched = True
            self.fw_state = FW_FAULT
            return self._accept(cmd)
        if cmd.msg_type == TYPE_STOP:
            self.targets = [0.0] * NUM_CHANNELS
            if any(self.applied):
                self.fw_state = FW_CONTROLLED_STOP
            return self._accept(cmd)
        if cmd.msg_type == TYPE_REARM:
            if self.estop_latched:
                self.estop_latched = False
                self.fw_state = FW_RECOVERY
            return self._accept(cmd)           # outputs stay zero; no motion implied
        if cmd.msg_type == TYPE_SETPOINT:
            if self.estop_latched:
                self.rejected_commands += 1
                return False                    # latched: not accepted
            self.targets = [v / 1000.0 for v in cmd.permille]
            if self.fw_state in (FW_BOOT, FW_RECOVERY, FW_IDLE, FW_ACTIVE, FW_CONTROLLED_STOP):
                self.fw_state = FW_ACTIVE if any(self.targets) else FW_IDLE
            return self._accept(cmd)
        self.rejected_commands += 1
        return False

    def _accept(self, cmd: DecodedCommand) -> bool:
        self.accepted_valid = True
        self.last_accepted_seq = cmd.sequence
        return True

    # ---- time ---------------------------------------------------------------------
    def step(self, now: float) -> None:
        if self._boot_time is None:
            self._boot_time = now
            self._last_cmd_time = now
        dt = 0.0 if self._last_step is None else max(0.0, now - self._last_step)
        self._last_step = now
        # command watchdog (the real one lives in firmware and its value is TBD)
        if now - self._last_cmd_time > self.command_timeout_s:
            self._timeout_active = True
            self.targets = [0.0] * NUM_CHANNELS
        self.applied = list(self.targets)
        for i in range(NUM_CHANNELS):
            ch = i + 1
            if ch in ENCODER_CHANNELS:
                self.encoder[i] += int(round(self.applied[i] * self._enc_rate * dt))
            if ch in ANGLE_CHANNELS:
                self.angle[i] += self.applied[i] * self._ang_rate * dt
        if self.fw_state == FW_BOOT:
            self.fw_state = FW_IDLE
        elif self.fw_state == FW_RECOVERY:
            self.fw_state = FW_IDLE
        elif self.fw_state == FW_CONTROLLED_STOP and not any(self.applied):
            self.fw_state = FW_IDLE
        elif self.fw_state == FW_ACTIVE and not any(self.applied):
            self.fw_state = FW_IDLE

    # ---- output -------------------------------------------------------------------
    def telemetry(self, now: float) -> TelemetryData:
        self.telemetry_seq = (self.telemetry_seq + 1) & 0xFFFFFFFF
        active = 0
        if self._timeout_active:
            active |= FAULT_COMMAND_TIMEOUT
        if self.estop_latched:
            active |= FAULT_ESTOP_LATCHED
        if self.invalid_channels:
            active |= FAULT_SENSOR_INVALID
        latched = FAULT_ESTOP_LATCHED if self.estop_latched else 0
        age = COMMAND_AGE_UNKNOWN
        if self._last_cmd_time is not None:
            age = min(65534, int(max(0.0, now - self._last_cmd_time) * 1000))
        channels = []
        for i in range(NUM_CHANNELS):
            ch = i + 1
            vf, ff = VALID_OUTPUT, 0
            rec = ChannelTelemetry(applied_output=self.applied[i])
            sensor_flag = (VALID_ENCODER if ch in ENCODER_CHANNELS else
                           VALID_ANGLE if ch in ANGLE_CHANNELS else VALID_CURRENT)
            if ch in self.invalid_channels:
                ff |= 1                              # SENSOR_INVALID; sensor valid bit stays clear
            else:
                vf |= sensor_flag
                if ch in ENCODER_CHANNELS:
                    rec.encoder_counts = self.encoder[i]
                elif ch in ANGLE_CHANNELS:
                    rec.joint_angle_rad = self.angle[i]
                else:
                    rec.current_a = abs(self.applied[i]) * self._cur_full
            rec.valid_flags, rec.fault_flags = vf, ff
            channels.append(rec)
        return TelemetryData(telemetry_seq=self.telemetry_seq, firmware_state=self.fw_state,
                             accepted_seq_valid=self.accepted_valid,
                             last_accepted_cmd_seq=self.last_accepted_seq, command_age_ms=age,
                             active_faults=active, latched_faults=latched, channels=channels)
