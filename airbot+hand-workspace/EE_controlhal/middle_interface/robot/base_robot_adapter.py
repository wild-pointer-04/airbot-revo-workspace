"""Abstract robot adapter. Concrete SDKs belong in robot/adapters/ only."""

from __future__ import annotations

from abc import ABC, abstractmethod

from protocol.pico_packet import Pose
from protocol.robot_command import RobotCommand


class BaseRobotAdapter(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def enable(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disable(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_ee_pose(self) -> Pose:
        raise NotImplementedError

    @abstractmethod
    def send_ee_target(self, command: RobotCommand) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    def go_home(self) -> None:
        """Move to the configured initial pose. Default is a no-op."""
        return None

    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError
