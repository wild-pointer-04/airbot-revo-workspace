"""Map Revo2 fingertip tactile forces to Manus glove vibration commands.

Pure helpers are ROS-free so unit tests do not need rclpy. The node entry
subscribes to ``control_msgs/DynamicJointState`` and publishes
``manus_ros2_msgs/ManusVibrationCommand``.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence


TACTILE_INTERFACE = "tactile_normal_force"

# ManusVibrationCommand finger order: Thumb, Index, Middle, Ring, Pinky.
FINGER_JOINT_SUFFIXES = (
    "thumb_proximal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)


def joint_names_for_side(side: str) -> tuple[str, ...]:
    if side not in ("left", "right"):
        raise ValueError("hand_side must be 'left' or 'right'")
    return tuple(f"{side}_{suffix}" for suffix in FINGER_JOINT_SUFFIXES)


def forces_from_dynamic_joint_state(
    message: Mapping[str, Any], side: str
) -> list[float] | None:
    """Extract five fingertip normal forces (N) from a DynamicJointState-like dict.

    Accepts both rclpy message attributes (via duck typing) and rosbridge dicts.
    Returns None when the message is incomplete.
    """
    expected = joint_names_for_side(side)
    joint_names = list(getattr(message, "joint_names", None) or message.get("joint_names") or [])
    interface_values = (
        getattr(message, "interface_values", None) or message.get("interface_values") or []
    )
    if len(joint_names) != len(interface_values):
        return None

    index_by_name = {str(name): idx for idx, name in enumerate(joint_names)}
    forces: list[float] = []
    for name in expected:
        idx = index_by_name.get(name)
        if idx is None:
            return None
        entry = interface_values[idx]
        if isinstance(entry, Mapping):
            names = list(entry.get("interface_names") or [])
            values = list(entry.get("values") or [])
        else:
            names = list(getattr(entry, "interface_names", []) or [])
            values = list(getattr(entry, "values", []) or [])
        if TACTILE_INTERFACE not in names:
            return None
        force_idx = names.index(TACTILE_INTERFACE)
        if force_idx >= len(values):
            return None
        try:
            forces.append(float(values[force_idx]))
        except (TypeError, ValueError):
            return None
    return forces


def forces_to_intensities(
    forces_n: Sequence[float],
    *,
    force_scale: float = 5.0,
    deadband_n: float = 0.05,
) -> list[float]:
    """Map Newtons to Manus vibration intensities in [0, 1]."""
    if force_scale <= 0:
        raise ValueError("force_scale must be positive")
    if deadband_n < 0:
        raise ValueError("deadband_n must be non-negative")
    out: list[float] = []
    for force in forces_n:
        value = float(force)
        if value < deadband_n:
            out.append(0.0)
            continue
        intensity = value / force_scale
        if intensity < 0.0:
            intensity = 0.0
        elif intensity > 1.0:
            intensity = 1.0
        out.append(intensity)
    return out


def main(argv: list[str] | None = None) -> int:
    del argv
    import rclpy
    from rclpy.node import Node
    from control_msgs.msg import DynamicJointState
    from manus_ros2_msgs.msg import ManusVibrationCommand

    class TactileToVibration(Node):
        def __init__(self) -> None:
            super().__init__("tactile_to_vibration")
            self.declare_parameter("hand_side", "right")
            self.declare_parameter("glove_index", 0)
            self.declare_parameter("force_scale", 5.0)
            self.declare_parameter("deadband_n", 0.05)
            self.declare_parameter("publish_hz", 30.0)

            self._side = str(self.get_parameter("hand_side").value)
            glove_index = int(self.get_parameter("glove_index").value)
            self._force_scale = float(self.get_parameter("force_scale").value)
            self._deadband_n = float(self.get_parameter("deadband_n").value)
            publish_hz = float(self.get_parameter("publish_hz").value)
            if publish_hz <= 0:
                raise ValueError("publish_hz must be positive")

            topic_in = f"/revo2_{self._side}/revo2_joint_state/dynamic_joint_states"
            topic_out = f"/manus_glove_{glove_index}/vibration_cmd"
            self._latest: list[float] | None = None
            self._sub = self.create_subscription(
                DynamicJointState, topic_in, self._on_dynamic, 10
            )
            self._pub = self.create_publisher(ManusVibrationCommand, topic_out, 10)
            self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)
            self.get_logger().info(
                f"Bridging {topic_in} → {topic_out} "
                f"(scale={self._force_scale}, deadband={self._deadband_n})"
            )

        def _on_dynamic(self, msg: DynamicJointState) -> None:
            forces = forces_from_dynamic_joint_state(msg, self._side)
            if forces is not None:
                self._latest = forces

        def _on_timer(self) -> None:
            if self._latest is None:
                return
            intensities = forces_to_intensities(
                self._latest,
                force_scale=self._force_scale,
                deadband_n=self._deadband_n,
            )
            cmd = ManusVibrationCommand()
            cmd.intensities = intensities
            self._pub.publish(cmd)

    rclpy.init()
    node = TactileToVibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
