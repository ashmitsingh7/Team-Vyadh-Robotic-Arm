"""fake_stm32: a pseudo-terminal that speaks the draft serial protocol (NOT firmware).

Lets you run the REAL serial_arm_bridge on a machine with no STM32:

    python3 -m arm_hardware.fake_stm32            # prints e.g. "slave port: /dev/pts/5"
    ros2 launch arm_bringup hardware_control.launch.py serial_port:=/dev/pts/5

It uses FirmwareSimModel (simulated behaviour, no ramping) and the same protocol code as the
bridge, so it validates framing/CRC/COBS/telemetry decoding and the bridge's link handling,
but it proves nothing about the real firmware or hardware. Fault injection options:
  --corrupt-rate R   corrupt a fraction R of telemetry frames (tests CRC counters)
  --version N        send telemetry with protocol version N (tests PROTOCOL_MISMATCH)
  --silent-after S   stop sending telemetry after S seconds (tests stale handling)
"""
import argparse
import os
import random
import select
import time
import tty
from typing import Optional

from arm_hardware import serial_protocol as sp
from arm_hardware import telemetry_parser as tp
from arm_hardware.firmware_sim import FirmwareSimModel


class FakeStm32:
    """Transport-independent protocol endpoint: feed bytes in, get bytes to send out."""

    def __init__(self, model: Optional[FirmwareSimModel] = None, version: int = sp.PROTOCOL_VERSION,
                 corrupt_rate: float = 0.0, rng: Optional[random.Random] = None):
        self.model = model or FirmwareSimModel()
        self.version = version
        self.corrupt_rate = corrupt_rate
        self._rng = rng or random.Random(0)
        self._decoder = sp.FrameDecoder()
        self.frames_ok = 0
        self.frames_bad = 0

    def receive(self, data: bytes, now: float) -> None:
        for item in self._decoder.feed(data):
            if isinstance(item, sp.FrameError):
                self.frames_bad += 1
                continue
            if item.version != sp.PROTOCOL_VERSION:
                self.frames_bad += 1       # firmware must not act on other versions
                continue
            try:
                cmd = sp.decode_command_frame(item)
            except sp.ProtocolError:
                self.frames_bad += 1
                continue
            self.frames_ok += 1
            self.model.handle_command(cmd, now)

    def telemetry_frame(self, now: float) -> bytes:
        self.model.step(now)
        payload = tp.build_telemetry_payload(self.model.telemetry(now))
        frame = bytearray(sp.encode_frame(sp.TYPE_TELEMETRY, payload, version=self.version))
        if self.corrupt_rate and self._rng.random() < self.corrupt_rate:
            i = self._rng.randrange(0, len(frame) - 1)
            frame[i] = (frame[i] ^ 0x5A) or 0x01     # never creates a delimiter
        return bytes(frame)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=20.0, help='telemetry rate in Hz (default 20)')
    ap.add_argument('--timeout', type=float, default=0.5, help='simulated command timeout in s')
    ap.add_argument('--corrupt-rate', type=float, default=0.0)
    ap.add_argument('--version', type=int, default=sp.PROTOCOL_VERSION)
    ap.add_argument('--silent-after', type=float, default=None)
    args = ap.parse_args(argv)

    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    print(f'fake STM32 (SIMULATED, not firmware) - slave port: {os.ttyname(slave)}', flush=True)
    dev = FakeStm32(FirmwareSimModel(command_timeout_s=args.timeout), args.version, args.corrupt_rate)
    start = time.monotonic()
    next_tx = start
    try:
        while True:
            now = time.monotonic()
            r, _, _ = select.select([master], [], [], max(0.0, next_tx - now))
            if r:
                try:
                    dev.receive(os.read(master, 512), time.monotonic())
                except OSError:
                    time.sleep(0.05)       # nobody has the slave open yet
            now = time.monotonic()
            if now >= next_tx:
                next_tx += 1.0 / args.rate
                if args.silent_after is None or now - start < args.silent_after:
                    try:
                        os.write(master, dev.telemetry_frame(now))
                    except OSError:
                        pass
    except KeyboardInterrupt:
        print(f'\nstopped. valid command frames: {dev.frames_ok}, rejected/bad: {dev.frames_bad}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
