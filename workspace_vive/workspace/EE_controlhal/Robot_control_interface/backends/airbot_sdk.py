"""AIRBOT Play SDK backend for the command-layer interface."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Mapping

import numpy as np

from arm_types import ArmCommand, ArmState, MotionMode
from convert import pose_to_xyzrpy, slew_pose
from interface import ArmController

logger = logging.getLogger(__name__)


class AirbotSdkController(ArmController):
    def __init__(self, robot: Any, config: Mapping[str, Any]) -> None:
        self._robot = robot
        self._joint_count = int(config.get("joint_count", 6))
        self._enable_eef = bool(config.get("enable_eef", False))
        self._publish_period = 1.0 / max(float(config.get("publish_hz", 100)), 1.0)
        self._slew_linear = float(config.get("slew_linear", 0.20))
        self._slew_angular = float(config.get("slew_angular", 0.80))
        self._speed_profile = str(config.get("speed_profile", "default")).lower()
        self._home_joints = np.asarray(config.get("home_joints", [0.0] * 6), dtype=float)
        if self._home_joints.shape != (self._joint_count,):
            raise ValueError("home_joints must contain exactly joint_count values")
        limits = config.get("joint_limits")
        self._joint_limits = None if limits is None else np.asarray(limits, dtype=float)
        if self._joint_limits is not None and self._joint_limits.shape != (self._joint_count, 2):
            raise ValueError("joint_limits must contain [min, max] for every joint")
        self._home_tolerance = float(config.get("home_tolerance", 0.03))
        self._lock = threading.Lock()
        self._connected = False
        self._running = False
        self._latest: ArmCommand | None = None
        self._stream_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._thread: threading.Thread | None = None
        self._state: ArmState | None = None
        self._mode = MotionMode.PROTECT

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "AirbotSdkController":
        cfg = dict(config or {})
        airbot = dict(cfg.get("airbot") or {})
        try:
            from airbot_py.arm import AIRBOTPlay
        except ImportError as exc:
            raise RuntimeError("airbot_py is unavailable; install the container SDK") from exc
        robot = AIRBOTPlay(url=str(airbot.get("url", "localhost")), port=int(airbot.get("port", 50051)))
        return cls(robot, airbot)

    def connect(self) -> None:
        if self._connected:
            return
        if not self._robot.connect():
            raise ConnectionError("AIRBOT SDK failed to connect")
        self._refresh_state()
        if self._state is None or self._state.joint_pos.size != self._joint_count:
            self.disconnect()
            raise RuntimeError(f"AIRBOT returned an unexpected joint count: {self._state}")
        from airbot_py.arm import RobotMode, SpeedProfile
        profiles = {
            "default": SpeedProfile.DEFAULT,
            "slow": SpeedProfile.SLOW,
            "fast": SpeedProfile.FAST,
        }
        if self._speed_profile not in profiles:
            raise ValueError(
                f"Unsupported AIRBOT speed_profile: {self._speed_profile!r}; "
                "expected default, slow, or fast"
            )
        self._robot.set_speed_profile(profiles[self._speed_profile])
        logger.info("AIRBOT speed profile: %s", self._speed_profile.upper())
        if not self._robot.switch_mode(RobotMode.SERVO_CART_POSE):
            self.disconnect()
            raise RuntimeError("AIRBOT failed to enter SERVO_CART_POSE mode")
        self._connected = True
        self._running = True
        self._thread = threading.Thread(target=self._command_loop, name="airbot-sdk", daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._running = False
        thread = self._thread
        if thread and thread.is_alive() and threading.current_thread() is not thread:
            thread.join(timeout=1.0)
        self._thread = None
        self._connected = False
        try:
            self._robot.disconnect()
        finally:
            self._state = None

    def _refresh_state(self) -> ArmState:
        joints = np.asarray(self._robot.get_joint_pos(), dtype=float).reshape(-1)
        pose = self._robot.get_end_pose()
        if pose is None or len(pose) != 2:
            raise RuntimeError("AIRBOT returned no end pose")
        position = np.asarray(pose[0], dtype=float).reshape(3)
        rotation = np.asarray(pose[1], dtype=float).reshape(4)
        state = ArmState(position, rotation, pose_to_xyzrpy(position, rotation), joints, True, self._mode, time.monotonic_ns())
        with self._lock:
            self._state = state
        return state

    def get_state(self) -> ArmState:
        if not self._connected:
            raise RuntimeError("AIRBOT is not connected")
        return self._refresh_state()

    def send_command(self, command: ArmCommand) -> None:
        if not self._connected:
            raise RuntimeError("AIRBOT is not connected")
        if not command.enabled:
            self.stop()
            return
        if command.gripper and not self._enable_eef:
            logger.info("Ignoring gripper command because enable_eef is false")
        with self._lock:
            self._latest = command
            if self._stream_pose is None and self._state is not None:
                self._stream_pose = (self._state.position.copy(), self._state.rotation.copy())
            self._mode = MotionMode.END_CONTROL

    def set_mode(self, mode: MotionMode) -> None:
        if not self._connected:
            raise RuntimeError("AIRBOT is not connected")
        from airbot_py.arm import RobotMode
        mode = MotionMode(int(mode))
        if mode == MotionMode.GRAVITY:
            with self._lock:
                self._latest = None
                self._mode = mode
            if not self._robot.switch_mode(RobotMode.GRAVITY_COMP):
                raise RuntimeError("AIRBOT failed to enter GRAVITY_COMP mode")
        elif mode == MotionMode.GO_HOME:
            with self._lock:
                self._latest = None
                self._mode = mode
            current = self._refresh_state().joint_pos
            self._validate_joint_limits("current state", current)
            self._validate_joint_limits("configured home_joints", self._home_joints)
            if not self._robot.switch_mode(RobotMode.PLANNING_POS):
                raise RuntimeError("AIRBOT failed to enter PLANNING_POS mode")
            if not self._robot.move_to_joint_pos(self._home_joints.tolist()):
                self.stop()
                raise RuntimeError("AIRBOT rejected the home motion")
            reached = self._refresh_state().joint_pos
            error = np.abs(reached - self._home_joints)
            if np.any(error > self._home_tolerance):
                self.stop()
                raise RuntimeError(
                    "AIRBOT reported home motion finished but did not reach the target; "
                    f"maximum joint error is {float(error.max()):.3f} rad"
                )
            reached_state = self._refresh_state()
            with self._lock:
                self._latest = None
                self._stream_pose = (
                    reached_state.position.copy(),
                    reached_state.rotation.copy(),
                )
        elif mode == MotionMode.END_CONTROL:
            if not self._robot.switch_mode(RobotMode.SERVO_CART_POSE):
                raise RuntimeError("AIRBOT failed to enter SERVO_CART_POSE mode")
        else:
            raise ValueError(f"Unsupported AIRBOT mode: {mode.name}")
        self._mode = mode

    def _validate_joint_limits(self, label: str, joints: np.ndarray) -> None:
        if self._joint_limits is None:
            return
        below = joints < self._joint_limits[:, 0]
        above = joints > self._joint_limits[:, 1]
        invalid = np.flatnonzero(below | above)
        if invalid.size == 0:
            return
        details = ", ".join(
            f"joint{index + 1}={joints[index]:.3f} not in "
            f"[{self._joint_limits[index, 0]:.3f}, {self._joint_limits[index, 1]:.3f}]"
            for index in invalid
        )
        raise RuntimeError(f"AIRBOT {label} is outside official planning limits: {details}")

    def stop(self) -> None:
        if not self._connected:
            return
        with self._lock:
            self._latest = None
            state = self._state
            if state is not None:
                self._stream_pose = (state.position.copy(), state.rotation.copy())
        if state is not None:
            from airbot_py.arm import RobotMode
            self._robot.switch_mode(RobotMode.SERVO_CART_POSE)
            self._mode = MotionMode.END_CONTROL

    def is_connected(self) -> bool:
        return self._connected

    def _command_loop(self) -> None:
        while self._running:
            with self._lock:
                latest = self._latest
                current = self._stream_pose
                mode = self._mode
            if latest is not None and mode == MotionMode.END_CONTROL:
                if current is None:
                    current = (latest.position.copy(), latest.rotation.copy())
                position, rotation = slew_pose(current[0], current[1], latest.position, latest.rotation, self._publish_period, self._slew_linear, self._slew_angular)
                with self._lock:
                    self._stream_pose = (position, rotation)
                self._robot.servo_cart_pose([position.tolist(), rotation.tolist()])
            try:
                self._refresh_state()
            except Exception:
                logger.exception("AIRBOT state refresh failed")
            time.sleep(self._publish_period)
