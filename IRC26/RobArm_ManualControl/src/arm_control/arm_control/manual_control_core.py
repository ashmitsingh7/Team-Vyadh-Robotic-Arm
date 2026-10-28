"""Manual-control state machine (pure Python, no ROS, no serial, no keyboard).

Turns validated operator intent into full seven-channel ArmCommand snapshots. The ROS node
(manual_control_node.py) is a thin wrapper that feeds it messages and a clock.

Responsibilities and non-responsibilities:
  * It requests direction + demand. It does NOT ramp: the STM32 owns ramping and the
    hardware command watchdog. Publishing steady snapshots at a fixed rate is a heartbeat,
    not a ramp.
  * ESTOP is a latched software REQUEST (not a physical emergency stop). It is cleared only
    by an explicit REARM; reconnects, restarts or fresh telemetry never clear or resume.
  * Each channel target expires if the operator input is not refreshed (backup deadman).
    This is NOT the hardware watchdog.
  * No automatic resume: a MOVE rejected because of ESTOP / REARM-in-progress / blocked
    state (or a target dropped for a blocked state) puts that channel in "hold-off". While
    the operator keeps sending MOVE for it, it stays rejected; only a release (or the input
    going quiet for input_timeout_s) clears it. So a key still held through a link loss or
    ESTOP cannot restart motion by itself.
  * Every outgoing command is re-validated; an internal error degrades to an all-zero
    SETPOINT, never to motion.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from arm_control.arm_state_monitor import StateAssessment
from arm_control.channels import CHANNEL_COUNT
from arm_control.command_mapper import MappingError, map_move
from arm_control.command_validation import (ACTION_ESTOP, ACTION_MOVE, ACTION_REARM, ACTION_STOP,
                                            COMMAND_TYPE_ESTOP, COMMAND_TYPE_REARM,
                                            COMMAND_TYPE_SETPOINT, COMMAND_TYPE_STOP,
                                            validate_arm_command, validate_user_input)
from arm_control.control_config import ControlConfig
from arm_control.software_limits import check_motion


@dataclass
class MotorCommandData:
    channel: int
    direction: int
    demand: float


@dataclass
class ArmCommandData:
    command_type: int
    sequence: int
    motors: List[MotorCommandData]


@dataclass
class InputResult:
    accepted: bool
    reason: str = ''


@dataclass
class Counters:
    inputs_received: int = 0
    inputs_rejected: int = 0
    commands_published: int = 0
    internal_errors: int = 0
    motion_blocks: int = 0


@dataclass
class _Target:
    direction: int
    demand: float
    expires: float


class ManualControlCore:
    def __init__(self, config: ControlConfig):
        self.cfg = config
        self.counters = Counters()
        self.estop_latched = False
        self._targets: Dict[int, _Target] = {}
        self._holdoff: Dict[int, float] = {}      # channel -> time a rejected MOVE was last seen
        self._rearm_left = 0
        self._stop_left = 0
        self._seq = 0
        self._last_block_reason: Optional[str] = None
        self._events: List[Tuple[str, str]] = []   # (level, text), drained by the node

    # ---- logging hook -----------------------------------------------------------
    def drain_events(self) -> List[Tuple[str, str]]:
        events, self._events = self._events, []
        return events

    def _log(self, level: str, text: str) -> None:
        self._events.append((level, text))

    # ---- helpers ----------------------------------------------------------------
    def _blocked(self, a: StateAssessment) -> Optional[str]:
        return a.blocked_reason if self.cfg.require_fresh_state else None

    def _reject(self, reason: str) -> InputResult:
        self.counters.inputs_rejected += 1
        self._log('warn', f'input rejected: {reason}')
        return InputResult(False, reason)

    @property
    def active_channels(self) -> List[int]:
        return sorted(self._targets)

    # ---- operator input ---------------------------------------------------------
    def handle_user_input(self, ui, now: float, a: StateAssessment) -> InputResult:
        self.counters.inputs_received += 1
        problems = validate_user_input(ui)
        if problems:
            return self._reject('; '.join(problems))
        action = ui.action

        if action == ACTION_ESTOP:
            if not self.estop_latched:
                self._log('error', 'EMERGENCY-STOP REQUEST latched. REARM required to continue '
                                   '(software request only; not a physical E-stop).')
            self.estop_latched = True
            self._targets.clear()
            self._rearm_left = 0
            self._stop_left = 0
            return InputResult(True, 'ESTOP latched')

        if action == ACTION_STOP:
            self._targets.clear()
            self._stop_left = self.cfg.stop_repeat
            return InputResult(True, 'STOP requested')

        if action == ACTION_REARM:
            if not self.estop_latched:
                return self._reject('REARM ignored: no ESTOP is latched')
            self.estop_latched = False
            self._targets.clear()
            self._stop_left = 0
            self._rearm_left = self.cfg.rearm_repeat
            self._log('warn', 'REARM requested: outputs stay zero; motion needs a new MOVE.')
            return InputResult(True, 'REARM requested')

        # ACTION_MOVE
        ch, direction = ui.channel, ui.direction
        ch_cfg = self.cfg.channels[ch]
        if direction == 0:                          # release: always allowed, clears hold-off
            self._targets.pop(ch, None)
            self._holdoff.pop(ch, None)
            return InputResult(True, 'released')
        if self.estop_latched:
            self._holdoff[ch] = now
            return self._reject('ESTOP latched: operator REARM required before any movement')
        if self._rearm_left > 0:
            self._holdoff[ch] = now
            return self._reject('REARM in progress; wait for it to finish')
        blocked = self._blocked(a)
        if blocked:
            self.counters.motion_blocks += 1
            self._holdoff[ch] = now
            return self._reject(f'motion blocked: {blocked}')
        if ch in self._holdoff:
            if now - self._holdoff[ch] <= self.cfg.input_timeout_s:
                self._holdoff[ch] = now
                return self._reject(f'M{ch}: release the key before moving again (motion was '
                                    'interrupted by ESTOP/blocked state; no automatic resume)')
            del self._holdoff[ch]
        try:
            target = map_move(ch, direction, ui.level, ch_cfg)
        except MappingError as exc:
            return self._reject(str(exc))
        limit = check_motion(ch, target.direction, ch_cfg, a.angles.get(ch))
        if limit:
            return self._reject(limit)
        self._stop_left = 0
        self._targets[ch] = _Target(target.direction, target.demand, now + self.cfg.input_timeout_s)
        return InputResult(True, 'target set')

    # ---- periodic command generation -------------------------------------------
    def tick(self, now: float, a: StateAssessment) -> ArmCommandData:
        """Produce the next ArmCommand snapshot (call at command_rate_hz)."""
        self._expire_and_recheck(now, a)
        if self.estop_latched:
            ctype = COMMAND_TYPE_ESTOP
        elif self._rearm_left > 0:
            ctype = COMMAND_TYPE_REARM
            self._rearm_left -= 1
        elif self._stop_left > 0 and not self._targets:
            ctype = COMMAND_TYPE_STOP
            self._stop_left -= 1
        else:
            ctype = COMMAND_TYPE_SETPOINT
        motors = []
        for ch in range(1, CHANNEL_COUNT + 1):
            t = self._targets.get(ch) if ctype == COMMAND_TYPE_SETPOINT else None
            motors.append(MotorCommandData(ch, t.direction if t else 0, t.demand if t else 0.0))
        cmd = ArmCommandData(ctype, self._seq, motors)
        problems = validate_arm_command(cmd)
        if problems:      # should be impossible; degrade to zero motion, never to movement
            self.counters.internal_errors += 1
            self._log('error', 'internal error, publishing zero SETPOINT: ' + '; '.join(problems))
            self._targets.clear()
            cmd = ArmCommandData(COMMAND_TYPE_SETPOINT, self._seq,
                                 [MotorCommandData(c, 0, 0.0) for c in range(1, CHANNEL_COUNT + 1)])
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        self.counters.commands_published += 1
        return cmd

    def _expire_and_recheck(self, now: float, a: StateAssessment) -> None:
        blocked = self._blocked(a)
        for ch in [c for c, seen in self._holdoff.items() if now - seen > self.cfg.input_timeout_s]:
            del self._holdoff[ch]                  # input went quiet: the key was really released
        if blocked and self._targets:
            self.counters.motion_blocks += 1
            for ch in self._targets:
                self._holdoff[ch] = now
            self._targets.clear()
        if blocked != self._last_block_reason:
            if blocked:
                self._log('warn', f'new motion blocked: {blocked}')
            elif self._last_block_reason:
                self._log('info', 'motion block cleared (state fresh and link healthy)')
            self._last_block_reason = blocked
        for ch in list(self._targets):
            t = self._targets[ch]
            if now >= t.expires:
                del self._targets[ch]
                self._log('info', f'M{ch} input expired (no refresh within {self.cfg.input_timeout_s} s)')
                continue
            limit = check_motion(ch, t.direction, self.cfg.channels[ch], a.angles.get(ch))
            if limit:
                del self._targets[ch]
                self._log('warn', f'M{ch} target dropped: {limit}')
