"""Arm command HAL. Teleop / VR stay outside this package."""

from __future__ import annotations

from abc import ABC, abstractmethod

from arm_types import ArmCommand, ArmState, MotionMode


class ArmController(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_state(self) -> ArmState:
        raise NotImplementedError

    @abstractmethod
    def send_command(self, command: ArmCommand) -> None:
        """Queue the latest absolute Cartesian target (latest-wins)."""
        raise NotImplementedError

    @abstractmethod
    def set_mode(self, mode: MotionMode) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """Hold the current TCP. Must not go SOFT (arm would drop)."""
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError
