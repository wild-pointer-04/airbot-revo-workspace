"""ARX X5 via ROS 2 /arm_cmd (mode=4) and /arm_status.

Requires:
  source /opt/ros/jazzy/setup.bash
  source /home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash
  X5Controller already running (bringup_single_arm.sh)

Must not open the Python CAN SDK at the same time.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Mapping

import numpy as np

from convert import pose_to_xyzrpy, slew_pose, xyzrpy_to_pose
from interface import ArmController
from arm_types import ArmCommand, ArmState, MotionMode

logger = logging.getLogger(__name__)

_ROS_HINT = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash"
)


class ArxRos2Controller(ArmController):
    def __init__(
        self,
        cmd_topic: str = "/arm_cmd",
        status_topic: str = "/arm_status",
        node_name: str = "robot_command_layer",
        publish_hz: float = 100.0,
        status_timeout_sec: float = 2.0,
        enable_gripper: bool = False,
        hold_mode: MotionMode = MotionMode.END_CONTROL,
        slew_linear: float = 0.50,
        slew_angular: float = 1.50,
    ) -> None:
        self.cmd_topic = cmd_topic
        self.status_topic = status_topic
        self.node_name = node_name
        self.publish_hz = max(float(publish_hz), 1.0)
        self.publish_period = 1.0 / self.publish_hz
        self.status_timeout_sec = float(status_timeout_sec)
        self.enable_gripper = bool(enable_gripper)
        self.hold_mode = MotionMode(int(hold_mode))
        self.slew_linear = float(slew_linear)
        self.slew_angular = float(slew_angular)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._node = None
        self._pub = None
        self._latest: ArmCommand | None = None
        self._stream_pos: np.ndarray | None = None
        self._stream_rot: np.ndarray | None = None
        self._tracking = False
        self._holding = False
        self._hold_joints: np.ndarray | None = None
        self._status: ArmState | None = None
        self._last_joint = np.zeros(6, dtype=np.float64)
        self._command_count = 0

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "ArxRos2Controller":
        cfg = dict(config or {})
        ros = dict(cfg.get("ros") or {})
        arm = dict(cfg.get("arm") or {})
        hold = str(arm.get("hold_mode", "end_control")).lower()
        if hold in {"protect", "2"}:
            hold_mode = MotionMode.PROTECT
        elif hold in {"joint", "5"}:
            hold_mode = MotionMode.JOINT
        else:
            hold_mode = MotionMode.END_CONTROL
        return cls(
            cmd_topic=str(ros.get("cmd_topic", "/arm_cmd")),
            status_topic=str(ros.get("status_topic", "/arm_status")),
            node_name=str(ros.get("node_name", "robot_command_layer")),
            publish_hz=float(ros.get("publish_hz", 100.0)),
            status_timeout_sec=float(ros.get("status_timeout_sec", 2.0)),
            enable_gripper=bool(arm.get("enable_gripper", False)),
            hold_mode=hold_mode,
            slew_linear=float(arm.get("slew_linear", 0.50)),
            slew_angular=float(arm.get("slew_angular", 1.50)),
        )

    def connect(self) -> None:
        from ros_env import ensure_ros

        ensure_ros()
        try:
            import rclpy
            from arx5_arm_msg.msg import RobotCmd, RobotStatus
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        except ImportError as exc:
            raise RuntimeError(
                f"rclpy / arx5_arm_msg not importable. {_ROS_HINT}"
            ) from exc

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self.node_name)
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self._node.create_publisher(RobotCmd, self.cmd_topic, qos)
        self._node.create_subscription(
            RobotStatus, self.status_topic, self._on_status, qos
        )
        self._node.create_timer(self.publish_period, self._on_timer)
        self._running = True
        self._thread = threading.Thread(target=self._spin, name="arx-ros2", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + self.status_timeout_sec
        while time.monotonic() < deadline:
            if self._status is not None:
                logger.info(
                    "[ARM] ARX ROS2 connected  status=%s  cmd=%s  tcp=%s",
                    self.status_topic,
                    self.cmd_topic,
                    np.round(self._status.position, 4).tolist(),
                )
                return
            time.sleep(0.02)
        self.disconnect()
        raise TimeoutError(
            f"no {self.status_topic} within {self.status_timeout_sec:.1f}s — "
            "start the arm with ./bringup_single_arm.sh"
        )

    def disconnect(self) -> None:
        self._running = False
        self._tracking = False
        self._holding = False
        thread = self._thread
        if thread is not None and thread.is_alive() and threading.current_thread() is not thread:
            thread.join(timeout=1.0)
        self._thread = None
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                logger.debug("node destroy failed", exc_info=True)
            self._node = None
        self._pub = None
        logger.info("[ARM] ARX ROS2 disconnected")

    def get_state(self) -> ArmState:
        with self._lock:
            if self._status is None:
                raise RuntimeError("no /arm_status yet")
            state = self._status
            return ArmState(
                position=state.position.copy(),
                rotation=state.rotation.copy(),
                xyzrpy=state.xyzrpy.copy(),
                joint_pos=state.joint_pos.copy(),
                connected=True,
                mode=state.mode,
                stamp_ns=state.stamp_ns,
            )

    def send_command(self, command: ArmCommand) -> None:
        if not self.is_connected():
            logger.warning("[ARM] send ignored: not connected")
            return
        if not command.enabled:
            self.stop()
            return
        with self._lock:
            entering = not self._tracking
            self._latest = command
            self._tracking = True
            self._holding = False
            self._hold_joints = None
            self._command_count += 1
            if entering:
                if self._status is not None:
                    self._stream_pos = self._status.position.copy()
                    self._stream_rot = self._status.rotation.copy()
                else:
                    self._stream_pos = command.position.copy()
                    self._stream_rot = command.rotation.copy()

    def set_mode(self, mode: MotionMode) -> None:
        mode = MotionMode(int(mode))
        if mode == MotionMode.GRAVITY:
            with self._lock:
                self._latest = None
                self._tracking = False
                self._holding = False
                self._hold_joints = None
            self._publish_joy(gravity=True, home=False)
            return
        if mode == MotionMode.GO_HOME:
            with self._lock:
                self._latest = None
                self._tracking = False
                self._holding = False
                self._hold_joints = None
            self._publish_joy(gravity=False, home=True)
            return
        with self._lock:
            state = self._status
        if state is None:
            return
        cmd = ArmCommand(
            position=state.position,
            rotation=state.rotation,
            mode=mode,
            enabled=True,
            source="set_mode",
        )
        if mode == MotionMode.END_CONTROL:
            self.send_command(cmd)
            return
        self._tracking = False
        self._publish_cmd(cmd)

    def stop(self) -> None:
        with self._lock:
            self._tracking = False
            if self._stream_pos is not None and self._stream_rot is not None:
                pos = self._stream_pos.copy()
                rot = self._stream_rot.copy()
            elif self._status is not None:
                pos = self._status.position.copy()
                rot = self._status.rotation.copy()
            else:
                pos = rot = None
            self._hold_joints = self._last_joint.copy()
            self._stream_pos = pos
            self._stream_rot = rot
            self._holding = pos is not None and rot is not None
        if pos is None or rot is None:
            return
        hold = ArmCommand(
            position=pos,
            rotation=rot,
            mode=self.hold_mode,
            enabled=True,
            source="stop",
        )
        joints = (
            self._hold_joints
            if self.hold_mode == MotionMode.JOINT
            else None
        )
        self._publish_cmd(hold, joints=joints)
        logger.info(
            "[ARM] hold %s at %s",
            self.hold_mode.name,
            np.round(pos, 4).tolist(),
        )

    def is_connected(self) -> bool:
        return self._running and self._status is not None

    def _spin(self) -> None:
        import rclpy

        while self._running and self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.005)
            except Exception:
                if self._running:
                    logger.exception("[ARM] rclpy spin failed")

    def _on_timer(self) -> None:
        with self._lock:
            if self._holding and not self._tracking:
                if self._stream_pos is None or self._stream_rot is None:
                    return
                command = ArmCommand(
                    position=self._stream_pos.copy(),
                    rotation=self._stream_rot.copy(),
                    mode=self.hold_mode,
                    enabled=True,
                    source="hold",
                )
                joints = (
                    None
                    if self._hold_joints is None or self.hold_mode != MotionMode.JOINT
                    else self._hold_joints.copy()
                )
            elif self._tracking and self._latest is not None:
                if self._stream_pos is None or self._stream_rot is None:
                    self._stream_pos = self._latest.position.copy()
                    self._stream_rot = self._latest.rotation.copy()
                pos, rot = slew_pose(
                    self._stream_pos,
                    self._stream_rot,
                    self._latest.position,
                    self._latest.rotation,
                    self.publish_period,
                    self.slew_linear,
                    self.slew_angular,
                )
                self._stream_pos = pos
                self._stream_rot = rot
                command = ArmCommand(
                    position=pos,
                    rotation=rot,
                    gripper=self._latest.gripper,
                    mode=MotionMode.END_CONTROL,
                    enabled=True,
                    source=self._latest.source,
                )
                joints = None
            else:
                return
        self._publish_cmd(command, joints=joints)

    def _on_status(self, msg) -> None:
        xyzrpy = np.asarray(msg.end_pos, dtype=np.float64).reshape(6)
        pos, rot = xyzrpy_to_pose(xyzrpy)
        joints = np.asarray(msg.joint_pos[:6], dtype=np.float64)
        with self._lock:
            self._last_joint = joints
            self._status = ArmState(
                position=pos,
                rotation=rot,
                xyzrpy=xyzrpy,
                joint_pos=joints,
                connected=True,
                stamp_ns=time.monotonic_ns(),
            )

    def _publish_cmd(
        self,
        command: ArmCommand,
        joints: np.ndarray | None = None,
    ) -> None:
        if self._pub is None or self._node is None:
            return
        from arx5_arm_msg.msg import RobotCmd

        if joints is None:
            if command.mode == MotionMode.END_CONTROL:
                joints = np.zeros(6, dtype=np.float64)
            else:
                joints = self._last_joint.copy()
        msg = RobotCmd()
        try:
            msg.header.stamp = self._node.get_clock().now().to_msg()
        except Exception:
            return
        xyzrpy = pose_to_xyzrpy(command.position, command.rotation)
        msg.end_pos = [float(v) for v in xyzrpy]
        msg.joint_pos = [float(v) for v in joints]
        msg.gripper = float(command.gripper) if self.enable_gripper else 0.0
        msg.mode = int(command.mode)
        try:
            self._pub.publish(msg)
        except Exception:
            logger.debug("publish failed", exc_info=True)

    def _publish_joy(self, gravity: bool, home: bool) -> None:
        if self._node is None:
            return
        from std_msgs.msg import Int32MultiArray

        pub = self._node.create_publisher(Int32MultiArray, "/arx_joy", 1)
        msg = Int32MultiArray()
        msg.data = [1 if gravity else 0, 1 if home else 0]
        pub.publish(msg)
        logger.info("[ARM] /arx_joy gravity=%s home=%s", gravity, home)
