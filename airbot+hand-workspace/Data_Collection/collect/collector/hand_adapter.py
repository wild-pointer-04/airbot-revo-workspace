"""Normalize Revo2 JointState (+ optional tactile) for unified LeRobot frames."""

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

# Fingertip tactile order matches Manus vibration: thumb, index, middle, ring, pinky.
TACTILE_JOINT_SUFFIXES = (
    "thumb_proximal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)
TACTILE_INTERFACE = "tactile_normal_force"
TACTILE_DIM = 5


def joint_names_for_side(side: str) -> tuple[str, ...]:
    if side not in ("left", "right"):
        raise ValueError("hand.side must be 'left' or 'right'")
    return tuple(f"{side}_{suffix}" for suffix in REVO2_JOINT_SUFFIXES)


def tactile_joint_names_for_side(side: str) -> tuple[str, ...]:
    if side not in ("left", "right"):
        raise ValueError("hand.side must be 'left' or 'right'")
    return tuple(f"{side}_{suffix}" for suffix in TACTILE_JOINT_SUFFIXES)


def missing_hand(side: str = "right") -> dict[str, np.ndarray]:
    prefix = f"observation.state.hand_{side}"
    return {
        f"{prefix}_joint": np.zeros(6, dtype=np.float32),
        f"{prefix}_velocity": np.zeros(6, dtype=np.float32),
        f"{prefix}_valid": np.zeros(1, dtype=np.float32),
        f"{prefix}_tactile": np.zeros(TACTILE_DIM, dtype=np.float32),
    }


def joint_state_to_observation(
    message: Mapping[str, Any], side: str = "right"
) -> dict[str, np.ndarray]:
    expected = joint_names_for_side(side)
    names = message.get("name")
    position = message.get("position")
    velocity = message.get("velocity")
    out = missing_hand(side)
    if not isinstance(names, (list, tuple)):
        return out
    if not isinstance(position, (list, tuple)) or not isinstance(velocity, (list, tuple)):
        return out
    if len(position) != len(names) or len(velocity) != len(names):
        return out

    index_by_name = {str(name): index for index, name in enumerate(names)}
    try:
        indexes = [index_by_name[name] for name in expected]
        positions = np.asarray([position[index] for index in indexes], dtype=np.float32)
        velocities = np.asarray([velocity[index] for index in indexes], dtype=np.float32)
    except (IndexError, KeyError, TypeError, ValueError):
        return out
    if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
        return out

    prefix = f"observation.state.hand_{side}"
    out[f"{prefix}_joint"] = positions
    out[f"{prefix}_velocity"] = velocities
    out[f"{prefix}_valid"] = np.ones(1, dtype=np.float32)
    return out


def tactile_from_dynamic_joint_state(
    message: Mapping[str, Any], side: str = "right"
) -> np.ndarray:
    """Return float32[5] fingertip normal forces, or zeros if incomplete."""
    expected = tactile_joint_names_for_side(side)
    joint_names = message.get("joint_names")
    interface_values = message.get("interface_values")
    zeros = np.zeros(TACTILE_DIM, dtype=np.float32)
    if not isinstance(joint_names, (list, tuple)) or not isinstance(interface_values, (list, tuple)):
        return zeros
    if len(joint_names) != len(interface_values):
        return zeros
    index_by_name = {str(name): idx for idx, name in enumerate(joint_names)}
    forces: list[float] = []
    try:
        for name in expected:
            idx = index_by_name[name]
            entry = interface_values[idx]
            if not isinstance(entry, Mapping):
                return zeros
            names = entry.get("interface_names") or []
            values = entry.get("values") or []
            if TACTILE_INTERFACE not in names:
                return zeros
            force_idx = list(names).index(TACTILE_INTERFACE)
            forces.append(float(values[force_idx]))
    except (KeyError, IndexError, TypeError, ValueError):
        return zeros
    arr = np.asarray(forces, dtype=np.float32)
    if arr.shape != (TACTILE_DIM,) or not np.isfinite(arr).all():
        return zeros
    return arr
