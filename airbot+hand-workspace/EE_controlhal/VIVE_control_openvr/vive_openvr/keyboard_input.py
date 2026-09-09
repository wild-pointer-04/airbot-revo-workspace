"""Teleop buttons: terminal keys + optional USB foot pedal (evdev).

Deadman (clutch) can be:
  - toggle from stdin (Space, legacy)
  - hold from pedal / key (press = on, release = off) — preferred for foot pedals

Left pedal on SayoDevice CM6K on this host emits KEY_1; confirm with
``scripts/probe_pedal.py`` if your firmware remapped it.
"""

from __future__ import annotations

import glob
import logging
import select
import sys
import termios
import threading
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

LOGGER = logging.getLogger(__name__)

DEFAULT_PEDAL_BY_ID = (
    "/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd"
)


@dataclass
class TeleopButtons:
    trigger: float = 0.0
    grip: float = 0.0
    home: bool = False
    pause: bool = False


def _resolve_pedal_device(pattern_or_path: str) -> Optional[str]:
    raw = str(pattern_or_path or "").strip()
    if not raw:
        return None
    path = Path(raw)
    if path.is_char_device() or path.exists():
        return str(path)
    matches = sorted(glob.glob(raw))
    for cand in matches:
        p = Path(cand)
        if p.exists():
            return str(p)
    # Fallback: any SayoDevice CM6K event-kbd
    for cand in sorted(glob.glob("/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd")):
        return cand
    return None


class KeyboardButtons:
    """Legacy stdin-only API (toggle Space / h / p / g)."""

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
        self._thread = threading.Thread(
            target=self._loop, name="openvr-vive-keys", daemon=True
        )
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
                    self._buttons.trigger = (
                        0.0 if self._buttons.trigger >= 0.5 else 1.0
                    )
            elif ch in {"h", "H"}:
                with self._lock:
                    self._buttons.home = True
            elif ch in {"p", "P"}:
                with self._lock:
                    self._buttons.pause = True
            elif ch in {"g", "G"}:
                with self._lock:
                    self._buttons.grip = 0.0 if self._buttons.grip >= 0.5 else 1.0


class TeleopButtonSource:
    """Stdin keys + optional foot-pedal hold for deadman."""

    def __init__(self, config: Optional[dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        self._lock = threading.Lock()
        self._buttons = TeleopButtons()
        self._stop = threading.Event()
        self._stdin_thread: threading.Thread | None = None
        self._pedal_thread: threading.Thread | None = None
        self._old_term = None
        self._fd = None

        self._stdin_enabled = bool(cfg.get("stdin", True))
        # Space remains toggle deadman for keyboard users.
        self._toggle_deadman_chars = set(str(c) for c in (cfg.get("toggle_deadman_chars") or [" "]))
        # Hold chars on stdin (rare; pedals usually go through evdev).
        self._hold_deadman_chars = set(str(c).lower() for c in (cfg.get("hold_deadman_chars") or []))

        pedal = dict(cfg.get("pedal") or {})
        self._pedal_enabled = bool(pedal.get("enabled", True))
        self._pedal_device_pat = str(
            pedal.get("device") or pedal.get("device_path") or DEFAULT_PEDAL_BY_ID
        )
        # Left pedal → deadman. Override after probe_pedal.py if needed.
        self._pedal_deadman_code = str(pedal.get("deadman_keycode") or "KEY_1").upper()
        if not self._pedal_deadman_code.startswith("KEY_"):
            self._pedal_deadman_code = f"KEY_{self._pedal_deadman_code}"
        mode = str(pedal.get("deadman_mode") or "hold").lower()
        self._pedal_hold = mode != "toggle"

    @classmethod
    def from_teleop_config(cls, teleop_config: dict[str, Any]) -> "TeleopButtonSource":
        buttons = dict(teleop_config.get("buttons") or {})
        return cls(buttons)

    def start(self) -> None:
        self._stop.clear()
        if self._stdin_enabled and sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
            self._old_term = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._stdin_thread = threading.Thread(
                target=self._stdin_loop, name="openvr-vive-keys", daemon=True
            )
            self._stdin_thread.start()
        if self._pedal_enabled:
            device = _resolve_pedal_device(self._pedal_device_pat)
            if device is None:
                LOGGER.warning(
                    "Foot pedal device not found (pattern=%s). "
                    "Deadman still works via Space. Run scripts/probe_pedal.py after fixing udev.",
                    self._pedal_device_pat,
                )
            else:
                self._pedal_thread = threading.Thread(
                    target=self._pedal_loop,
                    args=(device,),
                    name="openvr-vive-pedal",
                    daemon=True,
                )
                self._pedal_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._stdin_thread is not None:
            self._stdin_thread.join(timeout=1.0)
            self._stdin_thread = None
        if self._pedal_thread is not None:
            self._pedal_thread.join(timeout=1.0)
            self._pedal_thread = None
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

    def _set_trigger(self, value: float) -> None:
        with self._lock:
            self._buttons.trigger = float(value)

    def _toggle_trigger(self) -> None:
        with self._lock:
            self._buttons.trigger = 0.0 if self._buttons.trigger >= 0.5 else 1.0

    def _stdin_loop(self) -> None:
        assert self._fd is not None
        while not self._stop.is_set():
            ready, _, _ = select.select([self._fd], [], [], 0.05)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            if ch in self._toggle_deadman_chars:
                self._toggle_trigger()
            elif ch.lower() in self._hold_deadman_chars:
                # treat as toggle if only key-down arrives via tty
                self._toggle_trigger()
            elif ch in {"h", "H"}:
                with self._lock:
                    self._buttons.home = True
            elif ch in {"p", "P"}:
                with self._lock:
                    self._buttons.pause = True
            elif ch in {"g", "G"}:
                with self._lock:
                    self._buttons.grip = (
                        0.0 if self._buttons.grip >= 0.5 else 1.0
                    )

    def _pedal_loop(self, device_path: str) -> None:
        try:
            from evdev import InputDevice, categorize, ecodes
        except ImportError:
            LOGGER.warning("python-evdev not installed; pedal disabled (pip install evdev)")
            return
        try:
            dev = InputDevice(device_path)
        except PermissionError:
            LOGGER.error(
                "Cannot open pedal %s (permission denied). "
                "Install docs/99-sayodevice-pedal.rules or: sudo usermod -aG input $USER",
                device_path,
            )
            return
        except FileNotFoundError:
            LOGGER.error("Pedal device disappeared: %s", device_path)
            return

        LOGGER.info(
            "Foot pedal listening on %s (%s); deadman=%s mode=%s",
            device_path,
            getattr(dev, "name", "?"),
            self._pedal_deadman_code,
            "hold" if self._pedal_hold else "toggle",
        )
        try:
            for ev in dev.read_loop():
                if self._stop.is_set():
                    break
                if ev.type != ecodes.EV_KEY:
                    continue
                key = categorize(ev)
                code = key.keycode
                if isinstance(code, (list, tuple)):
                    codes = [str(c).upper() for c in code]
                else:
                    codes = [str(code).upper()]
                if self._pedal_deadman_code not in codes:
                    continue
                # keystate: 0=up 1=down 2=hold
                if self._pedal_hold:
                    if key.keystate == 1:
                        self._set_trigger(1.0)
                    elif key.keystate == 0:
                        self._set_trigger(0.0)
                else:
                    if key.keystate == 1:
                        self._toggle_trigger()
        except OSError as exc:
            LOGGER.warning("Pedal read ended: %s", exc)
