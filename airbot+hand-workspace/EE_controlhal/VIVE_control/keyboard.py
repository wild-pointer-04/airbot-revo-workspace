"""Keyboard stand-in for VIVE Tracker buttons.

Space toggles deadman (PICO index trigger).
h = go home (PICO right A).
p = pause (PICO left primary).
g toggles gripper.
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty
from dataclasses import dataclass


@dataclass
class TeleopButtons:
    trigger: float = 0.0
    grip: float = 0.0
    home: bool = False
    pause: bool = False


class KeyboardButtons:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._buttons = TeleopButtons()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._old_term = None
        self._fd = None

    def start(self) -> None:
        if not sys.stdin.isatty():
            return
        self._fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="vive-keys", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._fd is not None and self._old_term is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            except termios.error:
                pass
            self._fd = None
            self._old_term = None

    def snapshot(self) -> TeleopButtons:
        with self._lock:
            current = TeleopButtons(
                trigger=self._buttons.trigger,
                grip=self._buttons.grip,
                home=self._buttons.home,
                pause=self._buttons.pause,
            )
            self._buttons.home = False
            self._buttons.pause = False
            return current

    def set_buttons(
        self,
        *,
        trigger: float | None = None,
        grip: float | None = None,
        home: bool | None = None,
        pause: bool | None = None,
    ) -> None:
        with self._lock:
            if trigger is not None:
                self._buttons.trigger = float(trigger)
            if grip is not None:
                self._buttons.grip = float(grip)
            if home is not None:
                self._buttons.home = bool(home)
            if pause is not None:
                self._buttons.pause = bool(pause)

    def _loop(self) -> None:
        assert self._fd is not None
        while not self._stop.is_set():
            ready, _, _ = select.select([self._fd], [], [], 0.05)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            if ch == " ":
                with self._lock:
                    self._buttons.trigger = 0.0 if self._buttons.trigger >= 0.5 else 1.0
            elif ch in {"h", "H"}:
                with self._lock:
                    self._buttons.home = True
            elif ch in {"p", "P"}:
                with self._lock:
                    self._buttons.pause = True
            elif ch in {"g", "G"}:
                with self._lock:
                    self._buttons.grip = 0.0 if self._buttons.grip >= 0.5 else 1.0
