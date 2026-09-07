"""LeRobot feature schema: wrist camera + AIRBOT joint / EE pose."""

from __future__ import annotations

import numpy as np

CAM_SHAPE = (480, 640, 3)
DEFAULT_CAMERA_NAMES = ("cam_right_wrist",)
ARM_JOINT_DIM = 6


def camera_feature_key(name: str) -> str:
    return f"observation.images.{name}"


CAM_KEYS = tuple(camera_feature_key(n) for n in DEFAULT_CAMERA_NAMES)

_CAM_FEATURE = {
    "dtype": "video",
    "shape": CAM_SHAPE,
    "names": ["height", "width", "channels"],
}


def _vec(dtype: str, shape: tuple[int, ...], names: list[str] | None = None) -> dict:
    return {"dtype": dtype, "shape": shape, "names": names}


def build_features(camera_names: tuple[str, ...] | list[str] | None = None) -> dict:
    names = tuple(camera_names) if camera_names else DEFAULT_CAMERA_NAMES
    if not names:
        raise ValueError("at least one camera is required")

    features: dict = {camera_feature_key(name): dict(_CAM_FEATURE) for name in names}
    features.update(
        {
            "observation.host_monotonic_ns": _vec("int64", (1,), ["host_monotonic_ns"]),
            "observation.state.arm_joint": _vec(
                "float32", (ARM_JOINT_DIM,), [f"j{i}" for i in range(ARM_JOINT_DIM)]
            ),
            "observation.state.arm_ee_position": _vec("float32", (3,), ["x", "y", "z"]),
            "observation.state.arm_ee_orientation": _vec(
                "float32", (4,), ["qx", "qy", "qz", "qw"]
            ),
            "observation.state.arm_valid": _vec("float32", (1,), ["valid"]),
            "action": _vec(
                "float32", (ARM_JOINT_DIM,), [f"j{i}" for i in range(ARM_JOINT_DIM)]
            ),
        }
    )
    return features


def compose_action(frame: dict) -> np.ndarray:
    joints = np.asarray(
        frame.get("observation.state.arm_joint", np.zeros(ARM_JOINT_DIM)),
        dtype=np.float32,
    ).flatten()
    if joints.size != ARM_JOINT_DIM:
        out = np.zeros(ARM_JOINT_DIM, dtype=np.float32)
        out[: min(ARM_JOINT_DIM, joints.size)] = joints[:ARM_JOINT_DIM]
        return out
    return joints.astype(np.float32)


def black_image() -> np.ndarray:
    return np.zeros(CAM_SHAPE, dtype=np.uint8)
