from __future__ import annotations

import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from recorder import Revo2HandRecorder


def _config() -> dict:
    return {
        "hand": {"side": "right"},
        "recording_gate": {"topic": "/teleop/hand_recording", "max_age_s": 0.5},
    }


def test_recording_gate_requires_a_fresh_true_message() -> None:
    recorder = Revo2HandRecorder(_config())

    recorder._on_recording_gate({"data": True})

    assert recorder._recording_is_active(time.monotonic())
    recorder._recording_arrival_s = time.monotonic() - 0.51
    assert not recorder._recording_is_active(time.monotonic())


def test_save_episode_resets_only_the_current_episode_counter() -> None:
    class Dataset:
        saved = 0

        def has_pending_frames(self) -> bool:
            return True

        def save_episode(self) -> None:
            self.saved += 1

    recorder = Revo2HandRecorder(_config())
    recorder._dataset = Dataset()
    recorder._episode_frame_count = 3
    recorder._total_frame_count = 7

    recorder._save_episode()

    assert recorder._dataset.saved == 1
    assert recorder._episode_frame_count == 0
    assert recorder._total_frame_count == 7


def test_save_episode_ignores_an_empty_lerobot_buffer() -> None:
    class Dataset:
        saved = 0

        def has_pending_frames(self) -> bool:
            return False

        def save_episode(self) -> None:
            self.saved += 1

    recorder = Revo2HandRecorder(_config())
    recorder._dataset = Dataset()

    recorder._save_episode()

    assert recorder._dataset.saved == 0
