"""Normalize Revo2 ``sensor_msgs/JointState`` feedback for LeRobot frames."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np


REVO2_JOINT_SUFFIXES = (
    "thumb_proximal_joint",
    "thumb_metacarpal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)


def joint_names_for_side(side: str) -> tuple[str, ...]:
    """Return the driver-defined, stable order for one Revo2 hand."""
    if side not in ("left", "right"):
        raise ValueError("hand.side must be 'left' or 'right'")
    return tuple(f"{side}_{suffix}" for suffix in REVO2_JOINT_SUFFIXES)


def feature_schema(side: str) -> dict[str, dict[str, Any]]:
    """LeRobot features for physical feedback only; there is intentionally no action."""
    names = list(joint_names_for_side(side))
    prefix = f"observation.state.hand_{side}"
    return {
        "observation.host_monotonic_ns": {
            "dtype": "int64",
            "shape": (1,),
            "names": ["host_monotonic_ns"],
        },
        f"{prefix}_joint": {"dtype": "float32", "shape": (6,), "names": names},
        f"{prefix}_velocity": {
            "dtype": "float32",
            "shape": (6,),
            "names": names,
        },
        f"{prefix}_valid": {"dtype": "float32", "shape": (1,), "names": ["valid"]},
    }


def missing_state(side: str) -> dict[str, np.ndarray]:
    """A fixed-size invalid frame keeps the Parquet schema stable."""
    prefix = f"observation.state.hand_{side}"
    return {
        f"{prefix}_joint": np.zeros(6, dtype=np.float32),
        f"{prefix}_velocity": np.zeros(6, dtype=np.float32),
        f"{prefix}_valid": np.zeros(1, dtype=np.float32),
    }


def joint_state_to_observation(
    message: Mapping[str, Any], side: str
) -> dict[str, np.ndarray]:
    """Reorder a ROS JointState by name and reject incomplete numeric feedback.

    ``JointState`` ordering is not a transport contract. The Revo2 joint order is
    defined here once, then every recorded frame uses that order.
    """
    expected = joint_names_for_side(side)
    names = message.get("name")
    position = message.get("position")
    velocity = message.get("velocity")
    if not isinstance(names, (list, tuple)):
        return missing_state(side)
    if not isinstance(position, (list, tuple)) or not isinstance(velocity, (list, tuple)):
        return missing_state(side)
    if len(position) != len(names) or len(velocity) != len(names):
        return missing_state(side)

    index_by_name = {str(name): index for index, name in enumerate(names)}
    try:
        indexes = [index_by_name[name] for name in expected]
        positions = np.asarray([position[index] for index in indexes], dtype=np.float32)
        velocities = np.asarray([velocity[index] for index in indexes], dtype=np.float32)
    except (IndexError, KeyError, TypeError, ValueError):
        return missing_state(side)
    if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
        return missing_state(side)

    prefix = f"observation.state.hand_{side}"
    return {
        f"{prefix}_joint": positions,
        f"{prefix}_velocity": velocities,
        f"{prefix}_valid": np.ones(1, dtype=np.float32),
    }
