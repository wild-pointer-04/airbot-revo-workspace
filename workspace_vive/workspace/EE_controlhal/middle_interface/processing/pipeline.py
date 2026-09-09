"""Teleop processing pipeline. Output is RobotCommand or a stop request."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Mapping

from processing.clutch import Clutch
from processing.coordinate_transform import CoordinateTransform
from processing.filter import PoseFilter
from processing.head_base import HeadBase
from processing.safety import SafetyReason, SafetyStateMachine, TeleopState
from processing.velocity_limit import VelocityLimit
from processing.vr_normalize import to_teleop_pose
from processing.workspace_limit import WorkspaceLimit
from protocol.pico_packet import PicoTeleopPacket, Pose
from protocol.robot_command import RobotCommand

logger = logging.getLogger(__name__)


@dataclass
class PipelineResult:
    state: TeleopState
    command: RobotCommand | None
    should_stop: bool
    reason: SafetyReason
    clutch_active: bool
    tracking_ok: bool
    baseline: Pose | None = None
    delta_pos: np.ndarray | None = None
    should_go_home: bool = False


class TeleopPipeline:
    def __init__(self, config: Mapping[str, Any], get_robot_pose) -> None:
        self._config = config
        teleop = config.get("teleop") or {}
        filt = config.get("filter") or {}
        limits = config.get("limits") or {}
        transform_cfg = config.get("transform") or {}
        self.controller = str(teleop.get("controller", "right"))
        self.deadman_source = str(teleop.get("deadman", "trigger"))
        self.require_deadman_release_on_start = bool(
            teleop.get("require_deadman_release_on_start", False)
        )
        self._deadman_release_threshold = float(
            teleop.get("deadman_release_threshold", 0.3)
        )
        self._deadman_armed = not self.require_deadman_release_on_start
        self.gripper_source = str(teleop.get("gripper", "grip"))
        self.gripper_full_close = float(teleop.get("gripper_full_close", 0.85))
        self.pause_button = str(teleop.get("pause_button", "left_primary"))
        self.home_button = str(teleop.get("home_button", "right_primary"))
        self.vr_space = str(transform_cfg.get("vr_space", "unity"))
        self.head_base_enabled = bool(teleop.get("head_base", True))
        self.stop_on_workspace = bool(
            (config.get("workspace") or {}).get("stop_on_exceed", False)
        )
        self._get_robot_pose = get_robot_pose
        self.command_transform = CoordinateTransform.from_config(transform_cfg)
        self.head_base = HeadBase.from_config(teleop)
        self.clutch = Clutch(
            press_threshold=float(
                teleop.get(
                    "deadman_press_threshold",
                    teleop.get("grip_press_threshold", 0.5),
                )
            ),
            release_threshold=float(
                teleop.get(
                    "deadman_release_threshold",
                    teleop.get("grip_release_threshold", 0.3),
                )
            ),
            position_scale=float(teleop.get("position_scale", 1.0)),
            max_delta_position=float(teleop.get("max_delta_position", 0.30)),
            max_delta_angle=float(teleop.get("max_delta_angle", 1.0)),
            delay_vr_baseline=bool(teleop.get("delay_vr_baseline", True)),
            command_transform=self.command_transform,
        )
        self.filt = PoseFilter(
            enabled=bool(filt.get("enabled", False)),
            position_alpha=float(filt.get("position_alpha", 0.25)),
            rotation_alpha=float(filt.get("rotation_alpha", 0.25)),
        )
        self.velocity_enabled = bool(limits.get("enabled", False))
        self.velocity = VelocityLimit(
            max_linear_velocity=float(limits.get("max_linear_velocity", 0.20)),
            max_angular_velocity=float(limits.get("max_angular_velocity", 0.50)),
        )
        self.workspace = WorkspaceLimit.from_config(config.get("workspace"))
        self.safety = SafetyStateMachine()
        self._last_state = self.safety.state
        self._home_held = False

    def on_timeout(self) -> PipelineResult:
        self.clutch.reset()
        self.filt.reset()
        self.velocity.reset()
        self.head_base.reset()
        self._home_held = False
        self._deadman_armed = False
        if self.safety.state != TeleopState.DISCONNECTED:
            self.safety.trip(SafetyReason.TIMEOUT)
        else:
            self.safety.reason = SafetyReason.TIMEOUT
        self._log_state()
        return PipelineResult(
            state=self.safety.state,
            command=None,
            should_stop=True,
            reason=SafetyReason.TIMEOUT,
            clutch_active=False,
            tracking_ok=False,
        )

    def on_invalid_packet(self) -> PipelineResult:
        return PipelineResult(
            state=self.safety.state,
            command=None,
            should_stop=False,
            reason=SafetyReason.INVALID_PACKET,
            clutch_active=self.clutch.active,
            tracking_ok=False,
        )

    def process(self, packet: PicoTeleopPacket) -> PipelineResult:
        ctrl = packet.controller(self.controller)
        # In two-tracker mode, the body tracker defines the yaw frame.  Never
        # produce hand commands from HeadBase's last cached frame after it is
        # lost: that would make a later hand motion use a stale body heading.
        tracking_ok = bool(ctrl.valid) and (
            not self.head_base_enabled or bool(packet.hmd.valid)
        )
        prev = self.safety.state
        self.safety.on_connected()

        if self._home_edge(packet):
            self._reset_motion()
            self.safety.pause()
            logger.info("[TELEOP] Home requested → go_home_position")
            self._log_state(prev)
            return PipelineResult(
                self.safety.state,
                None,
                False,
                SafetyReason.GO_HOME,
                False,
                tracking_ok,
                should_go_home=True,
            )

        if self._pause_requested(packet) and self.safety.state == TeleopState.TELEOP_ACTIVE:
            self._reset_motion()
            self.safety.pause()
            self._log_state(prev)
            return PipelineResult(
                self.safety.state, None, True, SafetyReason.MANUAL_PAUSE, False, tracking_ok
            )

        if not tracking_ok:
            # Stop the backend's streaming target on the first lost frame.  A
            # hand-only packet is insufficient when HeadBase supplies yaw.
            # Requiring a deadman release prevents automatic resume when the
            # tracker comes back while the key/button is still held.
            should_stop = self.clutch.active or self.safety.state == TeleopState.TELEOP_ACTIVE
            self._reset_motion()
            self.safety.on_tracking_lost()
            self._log_state(prev)
            return PipelineResult(
                self.safety.state,
                None,
                should_stop,
                SafetyReason.TRACKING_LOST,
                False,
                False,
                None,
                None,
            )

        deadman = self._deadman(ctrl)
        if not self._deadman_armed:
            if deadman <= self._deadman_release_threshold:
                self._deadman_armed = True
                logger.info("[TELEOP] Deadman armed after release")
            return PipelineResult(
                self.safety.state,
                None,
                False,
                self.safety.reason,
                False,
                tracking_ok,
            )

        robot_pose = self._get_robot_pose()
        hand_rel = self._hand_rel(packet, ctrl.pose)
        clutch = self.clutch.update(deadman, hand_rel, robot_pose)

        if clutch.pressed_edge:
            logger.info("[TELEOP] Deadman pressed")
            self.velocity.reset()
            self.safety.on_clutch_pressed()
        if clutch.released_edge:
            logger.info("[TELEOP] Deadman released")
            self.velocity.reset()
            self.safety.on_clutch_released()

        if not clutch.active or clutch.target is None or not self.safety.allow_command():
            self._log_state(prev)
            return PipelineResult(
                self.safety.state,
                None,
                clutch.released_edge,
                self.safety.reason,
                clutch.active,
                tracking_ok,
                clutch.baseline,
                clutch.delta_pos,
            )

        if self.velocity_enabled:
            limited, _vel_exceeded = self.velocity.limit(clutch.target, time.monotonic())
        else:
            limited = clutch.target
        limited, ws_exceeded = self.workspace.clamp_pose(limited)

        if self.stop_on_workspace and ws_exceeded:
            self.clutch.reset()
            self.safety.trip(SafetyReason.WORKSPACE)
            self._log_state(prev)
            return PipelineResult(
                self.safety.state, None, True, SafetyReason.WORKSPACE, False, tracking_ok
            )

        command = RobotCommand(
            timestamp_ns=packet.receive_monotonic_ns or time.monotonic_ns(),
            position=limited.position,
            rotation=limited.rotation,
            gripper=self._gripper(ctrl),
            enabled=True,
            source=f"pico_{self.controller}",
        )
        self._log_state(prev)
        return PipelineResult(
            state=self.safety.state,
            command=command,
            should_stop=False,
            reason=SafetyReason.NONE,
            clutch_active=True,
            tracking_ok=True,
            baseline=clutch.baseline,
            delta_pos=clutch.delta_pos,
        )

    def _hand_rel(self, packet: PicoTeleopPacket, controller_pose: Pose) -> Pose:
        hand = to_teleop_pose(controller_pose, self.vr_space)
        if not self.head_base_enabled:
            return self.filt.update(hand)
        hmd = to_teleop_pose(packet.hmd.pose, self.vr_space)
        relative = self.head_base.relative(hmd, bool(packet.hmd.valid), hand)
        return self.filt.update(relative)

    def _deadman(self, ctrl) -> float:
        if self.deadman_source == "grip":
            return float(ctrl.grip)
        return float(ctrl.trigger)

    def _gripper(self, ctrl) -> float:
        value = float(ctrl.grip if self.gripper_source == "grip" else ctrl.trigger)
        if value >= self.gripper_full_close:
            return 1.0
        return value

    def _reset_motion(self) -> None:
        self.clutch.reset()
        self.filt.reset()
        self.velocity.reset()
        self._deadman_armed = False

    def _button_pressed(self, packet: PicoTeleopPacket, name: str) -> bool:
        if name in {"none", ""}:
            return False
        if name == "left_primary":
            return bool(packet.left_controller.primary_button)
        if name == "right_primary":
            return bool(packet.right_controller.primary_button)
        if name == "left_secondary":
            return bool(packet.left_controller.secondary_button)
        if name == "right_secondary":
            return bool(packet.right_controller.secondary_button)
        return False

    def _home_edge(self, packet: PicoTeleopPacket) -> bool:
        held = self._button_pressed(packet, self.home_button)
        edge = held and not self._home_held
        self._home_held = held
        return edge

    def _pause_requested(self, packet: PicoTeleopPacket) -> bool:
        return self._button_pressed(packet, self.pause_button)

    def _log_state(self, previous: TeleopState | None = None) -> None:
        if previous is None:
            previous = self._last_state
        if self.safety.state != previous:
            extra = ""
            if self.safety.reason != SafetyReason.NONE:
                extra = f" reason={self.safety.reason.value}"
            logger.info("[TELEOP] %s%s", self.safety.state.value, extra)
        self._last_state = self.safety.state
