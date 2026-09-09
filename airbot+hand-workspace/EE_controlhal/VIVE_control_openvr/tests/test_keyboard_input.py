from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_openvr.keyboard_input import TeleopButtonSource


def test_right_pedal_state_is_independent_from_the_arm_deadman() -> None:
    source = TeleopButtonSource(
        {"pedal": {"deadman_keycode": "KEY_1", "recording_keycode": "KEY_2"}}
    )

    source._set_trigger(1.0)
    source._set_recording(True)

    assert source.snapshot().trigger == 1.0
    assert source.recording_active()

    source._set_recording(False)
    assert not source.recording_active()
