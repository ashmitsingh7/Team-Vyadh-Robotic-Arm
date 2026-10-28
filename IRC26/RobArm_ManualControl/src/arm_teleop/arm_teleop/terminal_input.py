"""Terminal key source (cbreak mode) - the replaceable input layer.

Any object with a `read_keys() -> list[str]` method can replace this class (for example a
future evdev/pynput source that reports real key release). Dependencies: Python standard
library only (termios, tty, select). Limitation: no key-release events; works over SSH; needs
stdin to be a real terminal (not a pipe, not a launch-file child process).

Terminal settings are restored on normal exit, on exceptions and on Ctrl-C (cbreak mode keeps
Ctrl-C working). If the process is killed by SIGKILL the shell may need `reset`/`stty sane`.
"""
import os
import re
import select
import sys
import termios
import tty
from typing import List, Optional


class TerminalError(RuntimeError):
    """stdin is not a usable terminal."""


_ESCAPE_RE = re.compile(r'\x1b(?:\[[0-9;?]*[ -/]*[@-~]|O[@-~]|[@-Z\\-_])?')


def strip_escape_sequences(text: str) -> str:
    """Remove ANSI escape sequences (arrow keys etc.); a lone ESC is dropped too."""
    return _ESCAPE_RE.sub('', text)


class TerminalInput:
    def __init__(self, fd: Optional[int] = None):
        self._fd = sys.stdin.fileno() if fd is None else fd
        self._saved = None

    def __enter__(self) -> 'TerminalInput':
        if not os.isatty(self._fd):
            raise TerminalError('stdin is not a terminal; run the teleop node directly with '
                                '`ros2 run arm_teleop keyboard_teleop_node` in an interactive terminal '
                                '(it cannot be started as a launch-file child).')
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def restore(self) -> None:
        """Idempotent: put the terminal back the way we found it."""
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None

    def __exit__(self, exc_type, exc, tb) -> None:
        self.restore()

    def read_keys(self) -> List[str]:
        """Return all characters available right now without blocking."""
        keys: List[str] = []
        while select.select([self._fd], [], [], 0)[0]:
            data = os.read(self._fd, 256)
            if not data:
                break
            keys.extend(strip_escape_sequences(data.decode('utf-8', errors='ignore')))
        return keys
