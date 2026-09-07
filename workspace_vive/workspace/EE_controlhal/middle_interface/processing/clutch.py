"""Relative teleoperation: freeze robot TCP, apply head-base VR delta."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from processing.coordinate_transform import CoordinateTransform
from processing.math_pose import (
    as_quat,
    as_vec3,
    identity_quat,
    quat_angle,
    quat_apply,
    quat_relative,
    quat_slerp,
)
from protocol.pico_packet import Pose


@dataclass
class ClutchResult:
    active: bool
    pressed_edge: bool
    released_edge: bool
    target: Pose | None
    vr_baseline_ready: bool
    baseline: Pose | None = None
    delta_pos: np.ndarray | None = None


class Clutch:
    """Deadman rising edge latches robot TCP immediately, VR pose on the next frame.

    While held (teleop_standard delta, then optional teleop_to_command):
        Δp = scale * (p_now - p_vr_ref)   clamped to max_delta_position
        Δq = q_now * conj(q_vr_ref)       slerp-clamped to max_delta_angle
        cmd_pos = robot_ref.pos + R @ Δp
        cmd_rot = (R Δq R⁻¹) * robot_ref.rot
    """

    def __init__(
        self,
        press_threshold: float = 0.5,
        release_threshold: float = 0.3,
        position_scale: float = 1.0,
        max_delta_position: float = 0.30,
        max_delta_angle: float = 1.0,
        delay_vr_baseline: bool = True,
        command_transform: CoordinateTransform | None = None,
    ) -> None:
        if release_threshold > press_threshold:
            raise ValueError("release_threshold must be <= press_threshold")
        self.press_threshold = press_threshold
        self.release_threshold = release_threshold
        self.position_scale = position_scale
        self.max_delta_position = float(max_delta_position)
        self.max_delta_angle = float(max_delta_angle)
        self.delay_vr_baseline = bool(delay_vr_baseline)
        self.command_transform = command_transform
        self._pressed = False
        self._pending_vr = False
        self._controller_ref: Pose | None = None
        self._robot_ref: Pose | None = None

    @property
    def active(self) -> bool:
        return self._pressed

    @property
    def robot_ref(self) -> Pose | None:
        return None if self._robot_ref is None else self._robot_ref.copy()

    def reset(self) -> None:
        self._pressed = False
        self._pending_vr = False
        self._controller_ref = None
        self._robot_ref = None

    def update(self, deadman: float, controller: Pose, robot: Pose) -> ClutchResult:
        pressed_edge = False
        released_edge = False
        deadman = float(deadman)

        if not self._pressed and deadman >= self.press_threshold:
            self._pressed = True
            self._robot_ref = robot.copy()
            pressed_edge = True
            if self.delay_vr_baseline:
                self._controller_ref = None
                self._pending_vr = True
            else:
                self._controller_ref = controller.copy()
                self._pending_vr = False
        elif self._pressed and deadman <= self.release_threshold:
            self._pressed = False
            self._pending_vr = False
            self._controller_ref = None
            self._robot_ref = None
            released_edge = True

        if not self._pressed or self._robot_ref is None:
            return ClutchResult(False, pressed_edge, released_edge, None, False)

        zero = np.zeros(3, dtype=np.float64)
        if self._pending_vr or self._controller_ref is None:
            if pressed_edge and self.delay_vr_baseline:
                return ClutchResult(
                    True,
                    True,
                    False,
                    self._robot_ref.copy(),
                    False,
                    self._robot_ref.copy(),
                    zero,
                )
            self._controller_ref = controller.copy()
            self._pending_vr = False
            return ClutchResult(
                True,
                pressed_edge,
                False,
                self._robot_ref.copy(),
                True,
                self._robot_ref.copy(),
                zero,
            )

        target, delta_pos = self._compose_target(controller)
        return ClutchResult(
            True,
            pressed_edge,
            False,
            target,
            True,
            self._robot_ref.copy(),
            delta_pos,
        )

    def _compose_target(self, controller: Pose) -> tuple[Pose, np.ndarray]:
        assert self._controller_ref is not None and self._robot_ref is not None
        delta_pos = self.position_scale * (
            as_vec3(controller.position) - as_vec3(self._controller_ref.position)
        )
        dist = float(np.linalg.norm(delta_pos))
        if (
            self.max_delta_position > 0.0
            and dist > self.max_delta_position
            and dist > 1e-12
        ):
            delta_pos = delta_pos * (self.max_delta_position / dist)

        delta_rot = quat_relative(controller.rotation, self._controller_ref.rotation)
        angle = quat_angle(delta_rot)
        if self.max_delta_angle > 0.0 and angle > self.max_delta_angle and angle > 1e-12:
            delta_rot = quat_slerp(
                identity_quat(), delta_rot, self.max_delta_angle / angle
            )

        # DEBUG: 记录转换前后的增量
        import logging
        logger = logging.getLogger(__name__)
        logger.info(f"[CLUTCH DEBUG] delta_pos BEFORE transform: {delta_pos}")

        if self.command_transform is not None:
            delta_pos = self.command_transform.transform_position(delta_pos)
            delta_rot = self.command_transform.transform_rotation(delta_rot)
            logger.info(f"[CLUTCH DEBUG] delta_pos AFTER transform:  {delta_pos}")
            logger.info(f"[CLUTCH DEBUG] transform matrix R:\n{self.command_transform.R}")

        return (
            Pose(
                position=as_vec3(self._robot_ref.position) + delta_pos,
                rotation=as_quat(quat_apply(delta_rot, self._robot_ref.rotation)),
            ),
            delta_pos.copy(),
        )
