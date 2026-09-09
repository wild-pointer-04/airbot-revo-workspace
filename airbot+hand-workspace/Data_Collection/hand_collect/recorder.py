"""A bounded, physical-Revo2-feedback recorder using rosbridge and LeRobot."""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from hand_adapter import feature_schema, joint_state_to_observation, missing_state


log = logging.getLogger(__name__)

DATA_COLLECTION_ROOT = Path(__file__).resolve().parents[1]
VENDOR_LEROBOT_SRC = DATA_COLLECTION_ROOT / "vendor" / "lerobot-main" / "src"
DEFAULT_DATA_ROOT = DATA_COLLECTION_ROOT / "data" / "lerobot"


def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f"config must be a mapping: {path}")
    return config


class Revo2HandRecorder:
    """Collect pedal-gated physical Revo2 feedback as LeRobot episodes."""

    def __init__(self, config: dict[str, Any], data_root: str | Path | None = None):
        hand = dict(config.get("hand") or {})
        self._side = str(hand.get("side", "right"))
        self._topic_name = str(
            hand.get("topic")
            or f"/revo2_{self._side}/revo2_joint_state/joint_states"
        )
        self._max_age_s = float(hand.get("max_age_s", 0.2))
        if self._max_age_s <= 0:
            raise ValueError("hand.max_age_s must be positive")

        dataset = dict(config.get("dataset") or {})
        self._fps = int(dataset.get("fps", 10))
        if self._fps <= 0:
            raise ValueError("dataset.fps must be positive")
        self._repo_id = str(dataset.get("repo_id") or f"revo2_{self._side}_feedback")
        configured_root = dataset.get("root")
        self._dataset_path = (
            Path(data_root) / self._repo_id
            if data_root is not None
            else Path(configured_root).expanduser()
            if configured_root
            else DEFAULT_DATA_ROOT / self._repo_id
        )
        self._task = str(config.get("task") or "record Revo2 physical hand feedback")

        bridge = dict(config.get("rosbridge") or {})
        self._host = str(bridge.get("host", "localhost"))
        self._port = int(bridge.get("port", 9090))

        gate = dict(config.get("recording_gate") or {})
        self._recording_topic_name = str(
            gate.get("topic") or "/teleop/hand_recording"
        )
        self._recording_max_age_s = float(gate.get("max_age_s", 0.5))
        if self._recording_max_age_s <= 0:
            raise ValueError("recording_gate.max_age_s must be positive")

        self._latest = missing_state(self._side)
        self._latest_arrival_s: float | None = None
        self._latest_lock = threading.Lock()
        self._first_valid_state = threading.Event()
        self._dataset: Any = None
        self._ros: Any = None
        self._topic: Any = None
        self._recording_topic: Any = None
        self._recording_active = False
        self._recording_arrival_s: float | None = None
        self._recording_lock = threading.Lock()
        self._episode_frame_count = 0
        self._total_frame_count = 0

    def start(self) -> None:
        """Connect before allocating a dataset so unavailable ROS fails cleanly."""
        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError("Install hand_collect/requirements.txt first.") from exc

        self._ros = roslibpy.Ros(host=self._host, port=self._port)
        try:
            self._ros.run(timeout=10.0)
            connected = self._ros.is_connected
        except Exception:
            connected = False
        if not connected:
            self.close()
            raise RuntimeError(
                f"rosbridge is unavailable at ws://{self._host}:{self._port}. "
                "Start rosbridge in the Revo2 ROS environment first."
            )

        self._topic = roslibpy.Topic(
            self._ros,
            self._topic_name,
            "sensor_msgs/JointState",
            throttle_rate=0,
            queue_length=1,
        )
        self._topic.subscribe(self._on_joint_state)
        self._recording_topic = roslibpy.Topic(
            self._ros,
            self._recording_topic_name,
            "std_msgs/Bool",
            throttle_rate=0,
            queue_length=1,
        )
        self._recording_topic.subscribe(self._on_recording_gate)
        if not self._first_valid_state.wait(timeout=10.0):
            self.close()
            raise RuntimeError(
                f"No complete Revo2 feedback arrived on {self._topic_name} within 10 seconds. "
                "Verify the driver and source topic before recording."
            )
        self._dataset = self._open_dataset()
        log.info(
            "Subscribed to %s and %s; waiting for right pedal to record %s",
            self._topic_name,
            self._recording_topic_name,
            self._dataset_path,
        )

    def run(self, duration_s: float | None) -> int:
        """Write only while the fresh right-pedal gate is held."""
        self.start()
        deadline = None if duration_s is None else time.monotonic() + duration_s
        interval = 1.0 / self._fps
        next_tick = time.monotonic()
        was_recording = False
        try:
            while deadline is None or time.monotonic() < deadline:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(next_tick - now, 0.02))
                    continue
                recording = self._recording_is_active(now)
                if recording:
                    if not was_recording:
                        log.info("Right pedal pressed; starting a new episode")
                    self._write_frame()
                elif was_recording:
                    self._save_episode()
                was_recording = recording
                next_tick += interval
                if now - next_tick > interval:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            log.info("Interrupted; saving the current episode")
        finally:
            try:
                self._save_episode()
            finally:
                self.close()
        return self._total_frame_count

    def close(self) -> None:
        if self._recording_topic is not None:
            try:
                self._recording_topic.unsubscribe()
            except Exception:
                pass
            self._recording_topic = None
        if self._topic is not None:
            try:
                self._topic.unsubscribe()
            except Exception:
                pass
            self._topic = None
        if self._ros is not None:
            try:
                self._ros.terminate()
            except Exception:
                pass
            self._ros = None
        if self._dataset is not None:
            self._dataset.finalize()
            self._dataset = None

    def _on_joint_state(self, message: dict[str, Any]) -> None:
        state = joint_state_to_observation(message, self._side)
        with self._latest_lock:
            self._latest = state
            self._latest_arrival_s = time.monotonic()
        if state[f"observation.state.hand_{self._side}_valid"][0] == 1.0:
            self._first_valid_state.set()

    def _on_recording_gate(self, message: dict[str, Any]) -> None:
        active = message.get("data")
        if not isinstance(active, bool):
            return
        with self._recording_lock:
            self._recording_active = active
            self._recording_arrival_s = time.monotonic()

    def _recording_is_active(self, now_s: float) -> bool:
        with self._recording_lock:
            return bool(
                self._recording_active
                and self._recording_arrival_s is not None
                and now_s - self._recording_arrival_s <= self._recording_max_age_s
            )

    def _write_frame(self) -> None:
        with self._latest_lock:
            fresh = (
                self._latest_arrival_s is not None
                and time.monotonic() - self._latest_arrival_s <= self._max_age_s
            )
            state = self._latest if fresh else missing_state(self._side)
        frame = dict(state)
        frame["observation.host_monotonic_ns"] = np.asarray(
            [time.monotonic_ns()], dtype=np.int64
        )
        frame["task"] = self._task
        self._dataset.add_frame(frame)
        self._episode_frame_count += 1
        self._total_frame_count += 1

    def _save_episode(self) -> None:
        if self._dataset is None or not self._dataset.has_pending_frames():
            return
        self._dataset.save_episode()
        log.info(
            "Saved %d frames to %s",
            self._episode_frame_count,
            self._dataset_path,
        )
        self._episode_frame_count = 0

    def _open_dataset(self) -> Any:
        import sys

        source = str(VENDOR_LEROBOT_SRC)
        if source not in sys.path:
            sys.path.insert(0, source)
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise RuntimeError("LeRobot is unavailable; install hand_collect/requirements.txt.") from exc

        info_path = self._dataset_path / "meta" / "info.json"
        if info_path.is_file():
            return LeRobotDataset.resume(repo_id=self._repo_id, root=str(self._dataset_path))
        if self._dataset_path.exists():
            if any(self._dataset_path.iterdir()):
                raise RuntimeError(
                    f"Dataset path exists but is not a LeRobot dataset: {self._dataset_path}"
                )
            self._dataset_path.rmdir()
        self._dataset_path.parent.mkdir(parents=True, exist_ok=True)
        return LeRobotDataset.create(
            repo_id=self._repo_id,
            fps=self._fps,
            features=feature_schema(self._side),
            root=str(self._dataset_path),
            robot_type="revo2",
            use_videos=False,
        )
