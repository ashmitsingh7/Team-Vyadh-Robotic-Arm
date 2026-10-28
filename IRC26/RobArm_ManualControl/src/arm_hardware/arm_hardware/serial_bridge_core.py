"""Serial bridge logic (pure Python): link management, protocol, gating, diagnostics.

The ROS node (serial_arm_bridge.py) is a thin wrapper. This class talks to any "port"
object with non-blocking `read(n) -> bytes`, `write(data)` and `close()`; production passes a
pyserial port opened with timeout=0, tests pass a fake. Nothing here blocks for long.

Link states (ArmDiagnostics.LINK_*):
  DISCONNECTED       no port open (retrying every reconnect_interval_s)
  CONNECTING         port open, no valid telemetry yet
  CONNECTED          valid telemetry within telemetry_stale_after_s
  TELEMETRY_STALE    port open but no valid telemetry for telemetry_stale_after_s
  PROTOCOL_MISMATCH  telemetry with another protocol version; no SETPOINTs are sent

Safety behaviour: the bridge keeps no command memory. After every (re)connect, stale period
or mismatch the MotionGate is closed, so non-zero setpoints are replaced by zero until a zero
command has been seen. Nothing here is a watchdog: the STM32 owns the hardware command
timeout. "Transmitted" means bytes were handed to the port, nothing more.
"""
import math
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Callable, List, Optional, Tuple

from arm_control.command_validation import validate_arm_command
from arm_hardware.command_gate import (COMMAND_TYPE_ESTOP, COMMAND_TYPE_SETPOINT, COMMAND_TYPE_STOP,
                                       MotionGate)
from arm_hardware.bridge_types import (LINK_CONNECTED, LINK_CONNECTING, LINK_DISCONNECTED,
                                      LINK_PROTOCOL_MISMATCH, LINK_TELEMETRY_STALE, BridgeStats)
from arm_hardware.serial_protocol import (PROTOCOL_VERSION, Frame, FrameDecoder, FrameError,
                                          encode_command)
from arm_hardware.telemetry_parser import TelemetryData, TelemetryError, parse_telemetry_frame


@dataclass
class BridgeConfig:
    telemetry_stale_after_s: float = 0.5     # development default
    reconnect_interval_s: float = 1.0
    read_chunk: int = 512
    max_reads_per_poll: int = 16             # bounds work per poll so the executor is never starved


def _zero_setpoint(sequence: int):
    return SimpleNamespace(command_type=COMMAND_TYPE_SETPOINT, sequence=sequence,
                           motors=[SimpleNamespace(channel=c, direction=0, demand=0.0) for c in range(1, 8)])


class SerialBridgeCore:
    def __init__(self, opener: Callable[[], object], config: Optional[BridgeConfig] = None):
        self._opener = opener
        self.cfg = config or BridgeConfig()
        self.stats = BridgeStats(simulated=False)
        self.gate = MotionGate()
        self._port = None
        self._decoder = FrameDecoder()
        self._last_open_attempt: Optional[float] = None
        self._open_time: Optional[float] = None
        self._last_telemetry: Optional[float] = None
        self._ever_connected = False
        self._last_open_error: Optional[str] = None
        self._last_seq = 0
        self._events: List[Tuple[str, str]] = []

    # ---- events (drained by the node for logging) ----------------------------------
    def drain_events(self) -> List[Tuple[str, str]]:
        ev, self._events = self._events, []
        return ev

    def _log(self, level: str, text: str) -> None:
        self._events.append((level, text))

    @property
    def port_open(self) -> bool:
        return self._port is not None

    @property
    def link_state(self) -> int:
        return self.stats.link_state

    def _set_link(self, state: int, why: str = '') -> None:
        if state != self.stats.link_state:
            names = {0: 'DISCONNECTED', 1: 'CONNECTING', 2: 'CONNECTED', 3: 'TELEMETRY_STALE',
                     4: 'PROTOCOL_MISMATCH'}
            self._log('info' if state == LINK_CONNECTED else 'warn',
                      f'link {names[self.stats.link_state]} -> {names[state]}' + (f' ({why})' if why else ''))
            self.stats.link_state = state
            if state != LINK_CONNECTED:
                self.gate.close()      # never resume automatically after losing the link

    # ---- port management --------------------------------------------------------------
    def open(self, now: float) -> bool:
        """Try to open the port once. Returns success; failures are logged once per message."""
        self._last_open_attempt = now
        try:
            port = self._opener()
        except Exception as exc:  # noqa: BLE001 - any opener failure must be reported, not raised
            text = f'cannot open serial port: {exc}'
            if text != self._last_open_error:
                self._log('error', text)
                self._last_open_error = text
            self._set_link(LINK_DISCONNECTED)
            return False
        self._port = port
        self._open_time = now
        self._last_open_error = None
        self._decoder = FrameDecoder()
        self.gate.close()
        if self._ever_connected:
            self.stats.reconnect_count += 1
        self._ever_connected = True
        self.stats.remote_version_valid = False
        self._set_link(LINK_CONNECTING, 'port opened')
        return True

    def close(self, send_stop: bool = True) -> None:
        """Close the port (idempotent). Best-effort normal STOP first; the STM32 timeout is the
        real protection if this does not get through."""
        if self._port is None:
            return
        if send_stop:
            try:
                stop = SimpleNamespace(command_type=COMMAND_TYPE_STOP,
                                       sequence=(self._last_seq + 1) & 0xFFFFFFFF, motors=[])
                self._port.write(encode_command(stop))
            except Exception as exc:  # noqa: BLE001
                self._log('warn', f'could not send final STOP: {exc}')
        try:
            self._port.close()
        except Exception as exc:  # noqa: BLE001
            self._log('warn', f'error while closing port: {exc}')
        self._port = None
        self._set_link(LINK_DISCONNECTED, 'port closed')

    def _io_failure(self, exc: Exception) -> None:
        self._log('error', f'serial I/O error, closing port: {exc}')
        try:
            self._port.close()
        except Exception:  # noqa: BLE001
            pass
        self._port = None
        self._set_link(LINK_DISCONNECTED, 'I/O error')

    # ---- commands -----------------------------------------------------------------------
    def on_command(self, cmd, now: float) -> str:
        """Validate, gate and transmit one ArmCommand.

        Returns 'sent', 'sent_zero_substituted', 'rejected_invalid', 'blocked_mismatch'
        or 'not_connected'.
        """
        self.stats.commands_received += 1
        problems = validate_arm_command(cmd)
        if problems:
            self.stats.commands_rejected += 1
            self._log('warn', 'rejected invalid ArmCommand: ' + '; '.join(problems))
            return 'rejected_invalid'
        self._last_seq = int(cmd.sequence)
        if self._port is None:
            return 'not_connected'
        ctype = cmd.command_type
        if self.stats.link_state == LINK_PROTOCOL_MISMATCH and ctype == COMMAND_TYPE_SETPOINT:
            self.stats.commands_rejected += 1
            return 'blocked_mismatch'
        decision = self.gate.decide(cmd, self.stats.link_state == LINK_CONNECTED)
        outcome = 'sent'
        if decision.substitute_zero:
            self.stats.commands_rejected += 1
            self._log('warn', decision.reason)
            cmd = _zero_setpoint(self._last_seq)
            outcome = 'sent_zero_substituted'
        try:
            self._port.write(encode_command(cmd))
        except Exception as exc:  # noqa: BLE001
            self._io_failure(exc)
            return 'not_connected'
        self.stats.frames_transmitted += 1
        return outcome

    # ---- periodic polling ---------------------------------------------------------------
    def poll(self, now: float) -> List[TelemetryData]:
        """Reconnect if needed, read available bytes, decode telemetry, update link state."""
        if self._port is None:
            if (self._last_open_attempt is None
                    or now - self._last_open_attempt >= self.cfg.reconnect_interval_s):
                self.open(now)
            self._refresh_age(now)
            return []
        telemetry: List[TelemetryData] = []
        for _ in range(self.cfg.max_reads_per_poll):
            try:
                data = self._port.read(self.cfg.read_chunk)
            except Exception as exc:  # noqa: BLE001
                self._io_failure(exc)
                break
            if not data:
                break
            for item in self._decoder.feed(data):
                self._handle_decoded(item, now, telemetry)
            if len(data) < self.cfg.read_chunk:
                break
        self._check_stale(now)
        self._refresh_age(now)
        return telemetry

    def _handle_decoded(self, item, now: float, out: List[TelemetryData]) -> None:
        if isinstance(item, FrameError):
            if item.kind == 'crc':
                self.stats.frames_crc_error += 1
            else:
                self.stats.frames_malformed += 1
            return
        assert isinstance(item, Frame)
        try:
            data = parse_telemetry_frame(item)
        except TelemetryError as exc:
            if getattr(exc, 'version_mismatch', False):
                self.stats.remote_version_valid = True
                self.stats.remote_protocol_version = exc.remote_version
                self._set_link(LINK_PROTOCOL_MISMATCH,
                               f'firmware protocol version {exc.remote_version}, host {PROTOCOL_VERSION}')
            else:
                self.stats.frames_malformed += 1
            return
        self.stats.frames_received_valid += 1
        self.stats.remote_version_valid = True
        self.stats.remote_protocol_version = data.version
        self.stats.latched_faults = data.latched_faults
        self._last_telemetry = now
        self._set_link(LINK_CONNECTED)
        out.append(data)

    def _check_stale(self, now: float) -> None:
        if self._port is None or self.stats.link_state == LINK_PROTOCOL_MISMATCH:
            return
        reference = max(t for t in (self._last_telemetry, self._open_time) if t is not None)
        if (self.stats.link_state in (LINK_CONNECTED, LINK_CONNECTING)
                and now - reference > self.cfg.telemetry_stale_after_s):
            if self.stats.link_state == LINK_CONNECTED:
                self.stats.telemetry_timeouts += 1
            self._set_link(LINK_TELEMETRY_STALE, f'no valid telemetry for {self.cfg.telemetry_stale_after_s} s')

    def _refresh_age(self, now: float) -> None:
        self.stats.last_telemetry_age_s = (math.nan if self._last_telemetry is None
                                           else max(0.0, now - self._last_telemetry))
