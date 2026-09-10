from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

COLLECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COLLECT))

from collector.camera import CameraConfig, MultiCameraCapture, cameras_from_config
from collector.features import build_features
from collector.hand_adapter import (
    joint_state_to_observation,
    joint_names_for_side,
    tactile_from_dynamic_joint_state,
)


def test_build_features_includes_hand_and_tactile() -> None:
    feats = build_features(camera_names=["cam_global"], hand_side="right")
    assert "observation.images.cam_global" in feats
    assert feats["observation.state.hand_right_joint"]["shape"] == (6,)
    assert feats["observation.state.hand_right_tactile"]["shape"] == (5,)
    assert "action" in feats


def test_hand_joint_reorder() -> None:
    names = list(joint_names_for_side("right"))
    obs = joint_state_to_observation(
        {
            "name": list(reversed(names)),
            "position": [5, 4, 3, 2, 1, 0],
            "velocity": [50, 40, 30, 20, 10, 0],
        },
        "right",
    )
    assert obs["observation.state.hand_right_valid"][0] == 1.0
    np.testing.assert_array_equal(
        obs["observation.state.hand_right_joint"], np.arange(6, dtype=np.float32)
    )


def test_tactile_from_dynamic_joint_state() -> None:
    names = [
        "right_thumb_proximal_joint",
        "right_index_proximal_joint",
        "right_middle_proximal_joint",
        "right_ring_proximal_joint",
        "right_pinky_proximal_joint",
    ]
    msg = {
        "joint_names": names,
        "interface_values": [
            {"interface_names": ["tactile_normal_force"], "values": [float(i + 1)]}
            for i in range(5)
        ],
    }
    tactile = tactile_from_dynamic_joint_state(msg, "right")
    np.testing.assert_allclose(tactile, [1, 2, 3, 4, 5])


def test_opencv_camera_config_dispatch() -> None:
    caps = cameras_from_config(
        {
            "cam_global": {
                "type": "opencv",
                "device": "/dev/v4l/by-id/usb-LRCP_V1080P_LRCP_V1080P-video-index0",
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        },
        mock=True,
    )
    assert caps.camera_names == ["cam_global"]
    # mock=True forces MockStream regardless of type
    assert caps.start_all()["cam_global"] is True
    caps.stop_all()


def test_multicamera_opencv_backend_class() -> None:
    cfg = CameraConfig(
        name="cam",
        backend="opencv",
        device="/dev/null",
        width=64,
        height=48,
        fps=5,
    )
    capture = MultiCameraCapture([cfg], mock=False)
    assert capture._streams[0].__class__.__name__ == "OpenCVStream"
