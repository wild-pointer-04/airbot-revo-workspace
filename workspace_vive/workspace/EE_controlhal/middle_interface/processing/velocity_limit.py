"""Clamp end-effector step to configured linear / angular speed."""

from __future__ import annotations

import numpy as np

from processing.math_pose import as_quat, as_vec3, quat_angle, quat_relative, quat_slerp
from protocol.pico_packet import Pose


class VelocityLimit:
    def __init__(
        self,
        max_linear_velocity: float = 0.20,
        max_angular_velocity: float = 0.50,
    ) -> None:
        self.max_linear_velocity = float(max_linear_velocity)
        self.max_angular_velocity = float(max_angular_velocity)
        self._last: Pose | None = None
        self._last_time: float | None = None

    def reset(self) -> None:
        self._last = None
        self._last_time = None

    def limit(self, pose: Pose, now: float) -> tuple[Pose, bool]:
        """Returns (limited pose, raw_exceeded)."""
        if self._last is None or self._last_time is None:
            self._last = pose.copy()
            self._last_time = now
            return pose.copy(), False

        dt = now - self._last_time
        if dt <= 1e-6:
            return self._last.copy(), False

        delta = as_vec3(pose.position) - as_vec3(self._last.position)
        dist = float(np.linalg.norm(delta))
        max_dist = self.max_linear_velocity * dt
        exceeded = False
        if dist > max_dist and dist > 1e-12:
            exceeded = True
            delta = delta * (max_dist / dist)

        q_delta = quat_relative(pose.rotation, self._last.rotation)
        angle = quat_angle(q_delta)
        max_angle = self.max_angular_velocity * dt
        rot = as_quat(pose.rotation)
        if angle > max_angle and angle > 1e-12:
            exceeded = True
            t = max_angle / angle
            rot = quat_slerp(self._last.rotation, pose.rotation, t)

        limited = Pose(position=as_vec3(self._last.position) + delta, rotation=rot)
        self._last = limited.copy()
        self._last_time = now
        return limited, exceeded
