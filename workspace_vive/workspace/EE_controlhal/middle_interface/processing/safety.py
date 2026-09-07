"""Safety state machine for teleoperation."""

from __future__ import annotations

from enum import Enum


class TeleopState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    READY = "READY"
    TELEOP_ACTIVE = "TELEOP_ACTIVE"
    PAUSED = "PAUSED"
    ERROR = "ERROR"


class SafetyReason(str, Enum):
    NONE = "none"
    TIMEOUT = "timeout"
    TRACKING_LOST = "tracking_lost"
    INVALID_PACKET = "invalid_packet"
    WORKSPACE = "workspace"
    VELOCITY = "velocity"
    MANUAL_PAUSE = "manual_pause"
    GO_HOME = "go_home"


class SafetyStateMachine:
    """Never auto-starts motion. Clutch is the only enable path from READY/PAUSED."""

    def __init__(self) -> None:
        self.state = TeleopState.DISCONNECTED
        self.reason = SafetyReason.NONE

    def on_connected(self) -> TeleopState:
        if self.state in {TeleopState.DISCONNECTED, TeleopState.ERROR}:
            self.state = TeleopState.READY
            self.reason = SafetyReason.NONE
        return self.state

    def on_clutch_pressed(self) -> TeleopState:
        if self.state in {TeleopState.READY, TeleopState.PAUSED}:
            self.state = TeleopState.TELEOP_ACTIVE
            self.reason = SafetyReason.NONE
        return self.state

    def on_clutch_released(self) -> TeleopState:
        if self.state == TeleopState.TELEOP_ACTIVE:
            self.state = TeleopState.PAUSED
            self.reason = SafetyReason.NONE
        return self.state

    def trip(self, reason: SafetyReason) -> TeleopState:
        self.state = TeleopState.ERROR
        self.reason = reason
        return self.state

    def pause(self) -> TeleopState:
        self.state = TeleopState.PAUSED
        self.reason = SafetyReason.MANUAL_PAUSE
        return self.state

    def allow_command(self) -> bool:
        return self.state == TeleopState.TELEOP_ACTIVE
