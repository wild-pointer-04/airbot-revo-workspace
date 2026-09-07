"""Abstract robot adapter. Concrete SDKs belong in robot/adapters/ only."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod

import numpy as np

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

    def get_arm_state(self) -> dict:
        """Joint + EE pose for ROS2 / data collection. Quaternion is [x, y, z, w]."""
        pose = self.get_ee_pose()
        return {
            "observation.state.arm_joint": np.zeros(6, dtype=np.float32),
            "observation.state.arm_ee_position": np.asarray(pose.position, dtype=np.float32),
            "observation.state.arm_ee_orientation": np.asarray(pose.rotation, dtype=np.float32),
            "observation.state.arm_valid": np.array(
                [1.0 if self.is_connected() else 0.0], dtype=np.float32
            ),
            "host_monotonic_ns": time.monotonic_ns(),
        }
