"""Local mock / passthrough for AIRBOT state when ROS2 is not used."""

from __future__ import annotations

import time

import numpy as np

ARM_JOINT_DIM = 6


def missing_arm() -> dict[str, np.ndarray]:
    return {
        "observation.state.arm_joint": np.zeros(ARM_JOINT_DIM, dtype=np.float32),
        "observation.state.arm_ee_position": np.zeros(3, dtype=np.float32),
        "observation.state.arm_ee_orientation": np.array(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float32
        ),
        "observation.state.arm_valid": np.zeros(1, dtype=np.float32),
    }


class MockArmAdapter:
    """Sinusoidal joint / EE stand-in for --mock."""

    def start(self) -> None:
        self._t0 = time.monotonic()

    def stop(self) -> None:
        return None

    def read(self) -> dict[str, np.ndarray]:
        t = time.monotonic() - getattr(self, "_t0", time.monotonic())
        joints = 0.15 * np.sin(0.4 * t + np.arange(ARM_JOINT_DIM, dtype=np.float32))
        pos = np.array([0.30 + 0.02 * np.sin(t), 0.0, 0.20 + 0.01 * np.cos(t)], dtype=np.float32)
        return {
            "observation.state.arm_joint": joints.astype(np.float32),
            "observation.state.arm_ee_position": pos,
            "observation.state.arm_ee_orientation": np.array(
                [0.0, 0.0, 0.0, 1.0], dtype=np.float32
            ),
            "observation.state.arm_valid": np.array([1.0], dtype=np.float32),
            "host_monotonic_ns": time.monotonic_ns(),
        }
