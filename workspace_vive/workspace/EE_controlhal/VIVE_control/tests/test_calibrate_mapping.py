from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.calibrate_mapping import _resolve_tracker_config, _solve


def _sample(position: tuple[float, float, float]) -> dict:
    return {"ref": np.asarray(position, dtype=np.float64)}


def test_tracker_config_is_relative_to_vive_control_not_teleop_config_dir() -> None:
    assert _resolve_tracker_config({"tracker_config": "config/tracker.yaml"}) == (
        ROOT / "config" / "tracker.yaml"
    )


def test_standard_operator_motion_solves_the_airbot_command_basis() -> None:
    # After vr_normalize and HeadBase: forward/left/up are +X/+Y/+Z.
    recorded = {
        "baseline": _sample((0.0, 0.0, 0.0)),
        "up": _sample((0.0, 0.0, 0.30)),
        "forward": _sample((0.30, 0.0, 0.0)),
        "left": _sample((0.0, 0.30, 0.0)),
    }

    rotation, stats = _solve(recorded)

    np.testing.assert_allclose(
        rotation,
        [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    )
    assert all(stats[motion]["ok"] for motion in ("up", "forward", "left"))
