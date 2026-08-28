"""Pose filter. Rotation uses SLERP, never raw quaternion lerp without normalize."""

from __future__ import annotations

from protocol.pico_packet import Pose
from processing.math_pose import as_vec3, quat_slerp


class PoseFilter:
    def __init__(
        self,
        enabled: bool = True,
        position_alpha: float = 0.25,
        rotation_alpha: float = 0.25,
    ) -> None:
        self.enabled = enabled
        self.position_alpha = float(position_alpha)
        self.rotation_alpha = float(rotation_alpha)
        self._state: Pose | None = None

    def reset(self) -> None:
        self._state = None

    def update(self, pose: Pose) -> Pose:
        if not self.enabled:
            self._state = pose.copy()
            return pose.copy()
        if self._state is None:
            self._state = pose.copy()
            return pose.copy()
        pos = (1.0 - self.position_alpha) * as_vec3(self._state.position) + (
            self.position_alpha * as_vec3(pose.position)
        )
        rot = quat_slerp(self._state.rotation, pose.rotation, self.rotation_alpha)
        self._state = Pose(position=pos, rotation=rot)
        return self._state.copy()
