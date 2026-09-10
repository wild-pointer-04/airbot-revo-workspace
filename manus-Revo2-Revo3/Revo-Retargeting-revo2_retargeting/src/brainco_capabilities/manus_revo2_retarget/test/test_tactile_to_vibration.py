from __future__ import annotations

from manus_revo2_retarget.tactile_to_vibration import (
    forces_from_dynamic_joint_state,
    forces_to_intensities,
    joint_names_for_side,
)


def test_forces_reordered_by_joint_name() -> None:
    names = list(joint_names_for_side("right"))
    message = {
        "joint_names": list(reversed(names)),
        "interface_values": [
            {"interface_names": ["tactile_normal_force"], "values": [float(i)]}
            for i in reversed(range(5))
        ],
    }
    forces = forces_from_dynamic_joint_state(message, "right")
    assert forces == [0.0, 1.0, 2.0, 3.0, 4.0]


def test_missing_tactile_returns_none() -> None:
    names = list(joint_names_for_side("right"))
    message = {
        "joint_names": names,
        "interface_values": [
            {"interface_names": ["current"], "values": [1.0]} for _ in names
        ],
    }
    assert forces_from_dynamic_joint_state(message, "right") is None


def test_intensity_clip_and_deadband() -> None:
    assert forces_to_intensities([0.01, 2.5, 10.0], force_scale=5.0, deadband_n=0.05) == [
        0.0,
        0.5,
        1.0,
    ]
