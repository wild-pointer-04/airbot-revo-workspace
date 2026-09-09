from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hand_adapter import joint_names_for_side, joint_state_to_observation


def test_joint_state_is_reordered_by_name() -> None:
    names = list(joint_names_for_side("right"))
    message = {
        "name": list(reversed(names)),
        "position": [5, 4, 3, 2, 1, 0],
        "velocity": [50, 40, 30, 20, 10, 0],
    }

    observation = joint_state_to_observation(message, "right")

    assert observation["observation.state.hand_right_valid"][0] == 1.0
    np.testing.assert_array_equal(
        observation["observation.state.hand_right_joint"], np.arange(6, dtype=np.float32)
    )
    np.testing.assert_array_equal(
        observation["observation.state.hand_right_velocity"],
        np.arange(0, 60, 10, dtype=np.float32),
    )


def test_missing_joint_writes_a_fixed_invalid_state() -> None:
    names = list(joint_names_for_side("right"))[:-1]
    observation = joint_state_to_observation(
        {"name": names, "position": [0.0] * 5, "velocity": [0.0] * 5}, "right"
    )

    assert observation["observation.state.hand_right_valid"][0] == 0.0
    np.testing.assert_array_equal(
        observation["observation.state.hand_right_joint"], np.zeros(6, dtype=np.float32)
    )
