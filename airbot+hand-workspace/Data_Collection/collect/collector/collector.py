"""Episode orchestrator: 10 Hz LeRobot tick + keyboard /record_cmd."""

from __future__ import annotations

import logging
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

from collector.arm_adapter import MockArmAdapter, missing_arm
from collector.camera import cameras_from_config
from collector.features import black_image, camera_feature_key
from collector.lerobot_writer import LeRobotWriter
from collector.paths import DEFAULT_DATA_ROOT
from collector.ros2_codec import ARM_TOPIC
from collector.sync_buffer import SyncBuffer

log = logging.getLogger(__name__)

SRC_ARM = "arm"


def _observation_fields(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep LeRobot observation.* keys; drop ROS sidecar fields like host_monotonic_ns."""
    return {k: v for k, v in payload.items() if k.startswith("observation.")}


def load_collect_config(path: Path | str) -> dict[str, Any]:
    with Path(path).open() as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"collect config must be a mapping: {path}")
    return data


class Collector:
    def __init__(self, config: dict[str, Any], mock: bool = False):
        self._config = config
        self._mock = mock or bool(config.get("mock", False))

        self._data_root = Path(config.get("data_root") or DEFAULT_DATA_ROOT)
        self._data_root.mkdir(parents=True, exist_ok=True)
        self._task_id = str(config.get("task", "pick_cup"))
        ds = dict(config.get("dataset") or {})
        self._fps = int(ds.get("fps", 10))
        self._repo_id = str(ds.get("repo_id") or self._task_id)
        self._dataset_root = Path(ds.get("root") or (self._data_root / "lerobot"))
        self._dataset_path = self._dataset_root / self._repo_id

        self._recording = False
        self._saving = False
        self._running = True
        self._lock = threading.Lock()
        self._writer_lock = threading.Lock()
        self._frame_idx = 0
        self._episode_id: Optional[str] = None
        self._episode_start_ns = 0
        self._cameras = cameras_from_config(dict(config.get("cameras") or {}), mock=self._mock)
        self._camera_names = list(self._cameras.camera_names)
        if not self._camera_names:
            raise ValueError("collect.yaml cameras: enable at least one camera")
        self._cam_fail_streak = {name: 0 for name in self._camera_names}
        self._buffer = SyncBuffer(
            self._camera_names + [SRC_ARM],
            max_age_s=0.2,
            max_age_by_source={SRC_ARM: 0.2},
        )

        source = str(config.get("state_source", "ros2")).lower()
        if source not in ("ros2", "rosbridge", "local"):
            raise ValueError(
                f"state_source must be ros2, rosbridge or local, got {source!r}"
            )
        self._state_source = source
        self._arm = None
        self._ros_hub = None
        if source == "local":
            self._arm = MockArmAdapter()

        self._writer = LeRobotWriter(
            repo_id=self._repo_id,
            root=str(self._dataset_root),
            fps=self._fps,
            use_videos=bool(ds.get("use_videos", True)),
            robot_type="airbot",
            task=self._task_id,
            camera_names=self._camera_names,
        )
        self._ros = None
        self._poll_stop = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None

    def start_session(self) -> None:
        self._writer.initialize()
        cam_ok = self._cameras.start_all()
        log.info("Cameras start: %s", cam_ok)
        if self._state_source == "ros2":
            from collector.ros2_hub import Ros2StateHub

            ros_cfg = dict(self._config.get("ros2") or {})
            self._ros_hub = Ros2StateHub(
                self._buffer,
                arm_topic=str(ros_cfg.get("arm_topic") or ARM_TOPIC),
            )
            self._ros_hub.start()
        elif self._state_source == "rosbridge":
            from collector.rosbridge_hub import RosbridgeStateHub

            ros_cfg = dict(self._config.get("ros2") or {})
            bridge = dict(self._config.get("rosbridge") or {})
            self._ros_hub = RosbridgeStateHub(
                self._buffer,
                arm_topic=str(ros_cfg.get("arm_topic") or ARM_TOPIC),
                host=str(bridge.get("host", "localhost")),
                port=int(bridge.get("port", 9090)),
            )
            self._ros_hub.start()
        else:
            assert self._arm is not None
            self._arm.start()
            self._start_poller()
        self._maybe_connect_rosbridge()
        log.info(
            "Collector ready mock=%s state=%s task=%s fps=%d writer=%s",
            self._mock,
            self._state_source,
            self._task_id,
            self._fps,
            self._writer.backend,
        )

    def _start_poller(self) -> None:
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True, name="collect-poll")
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        interval = 1.0 / 50.0
        while not self._poll_stop.is_set():
            arm = self._arm.read()
            host_ns = arm.get("host_monotonic_ns")
            ts = (int(host_ns) / 1_000_000_000.0) if host_ns is not None else None
            self._buffer.update(SRC_ARM, arm, timestamp=ts)
            time.sleep(interval)

    def _maybe_connect_rosbridge(self) -> None:
        bridge = dict(self._config.get("rosbridge") or {})
        if not bridge.get("enabled", False):
            return
        try:
            import roslibpy
        except ImportError:
            log.warning("roslibpy not installed; /record_cmd disabled")
            return
        self._ros = roslibpy.Ros(
            host=str(bridge.get("host", "localhost")),
            port=int(bridge.get("port", 9090)),
        )
        self._ros.run()
        topic = roslibpy.Topic(self._ros, str(bridge.get("topic", "/record_cmd")), "std_msgs/String")
        topic.subscribe(self._record_cmd_cb)
        log.info("Subscribed to /record_cmd via rosbridge")

    def _record_cmd_cb(self, msg: dict) -> None:
        cmd = msg.get("data", "")
        if cmd == "start":
            self.start_recording()
        elif cmd == "stop":
            self.stop_recording()
        elif cmd == "save":
            self.save_episode()
        elif cmd == "discard":
            self.discard_episode()
        else:
            log.warning("Unknown record command: %s", cmd)

    def _record_tick(self) -> None:
        frames = self._cameras.get_frames()
        for name, frame in frames.items():
            if frame is not None:
                self._buffer.update(name, frame)

        with self._lock:
            if not self._recording:
                return
            snapshot = self._buffer.get_snapshot()
            host_ns = time.monotonic_ns()
            black = black_image()
            frame_data: dict[str, Any] = {
                "observation.host_monotonic_ns": np.array([host_ns], dtype=np.int64),
            }
            for src in self._camera_names:
                key = camera_feature_key(src)
                img = snapshot.get(src)
                if img is None:
                    self._cam_fail_streak[src] += 1
                    if self._cam_fail_streak[src] == 30:
                        log.error("Camera '%s' dropped 30 consecutive frames", src)
                    frame_data[key] = black
                else:
                    self._cam_fail_streak[src] = 0
                    frame_data[key] = img

            arm = snapshot.get(SRC_ARM) or missing_arm()
            frame_data.update(_observation_fields(arm))

            with self._writer_lock:
                self._writer.add_frame(frame_data)
            self._frame_idx += 1
            if self._frame_idx % (self._fps * 3) == 0:
                log.info("Recording... frame %d", self._frame_idx)

    def start_recording(self) -> None:
        with self._lock:
            if self._recording:
                return
            if self._saving:
                log.warning("Cannot start: save in progress")
                return
            with self._writer_lock:
                if self._writer.frame_count > 0:
                    log.warning("Clearing unsaved frames")
                    self._writer.discard_episode()
            self._episode_id = f"{self._writer.episode_count:06d}"
            self._episode_start_ns = time.monotonic_ns()
            self._recording = True
            self._frame_idx = 0
            self._cam_fail_streak = {name: 0 for name in self._camera_names}
        log.info("Recording STARTED episode=%s", self._episode_id)

    def stop_recording(self) -> None:
        with self._lock:
            if not self._recording:
                return
            self._recording = False
        log.info("Recording STOPPED (%d frames) episode=%s", self._frame_idx, self._episode_id)

    def save_episode(self) -> None:
        with self._lock:
            was = self._recording
            self._recording = False
            frame_count = self._frame_idx
        if frame_count == 0:
            log.warning("No frames recorded, nothing to save")
            return
        self._saving = True
        try:
            with self._writer_lock:
                self._writer.save_episode()
            log.info("Episode saved frames=%d", frame_count)
        except Exception as exc:
            log.error("Episode save FAILED: %s", exc)
            try:
                with self._writer_lock:
                    self._writer.discard_episode()
            except Exception:
                pass
        finally:
            self._saving = False
        with self._lock:
            self._frame_idx = 0
            self._episode_id = None

    def discard_episode(self) -> None:
        with self._lock:
            self._recording = False
            frame_count = self._frame_idx
        if frame_count == 0:
            log.info("No frames to discard")
            return
        with self._writer_lock:
            self._writer.discard_episode()
        log.info("Episode discarded")
        with self._lock:
            self._frame_idx = 0
            self._episode_id = None

    def run(self, seconds: Optional[float] = None) -> None:
        stop_event = threading.Event()

        def _sigint_handler(sig, frame):
            log.info("Signal received, shutting down...")
            stop_event.set()

        signal.signal(signal.SIGINT, _sigint_handler)
        signal.signal(signal.SIGTERM, _sigint_handler)

        deadline: Optional[float] = None
        if seconds is not None:
            time.sleep(0.05)
            self.start_recording()
            deadline = time.monotonic() + max(seconds, 0.05)
        else:
            kb = threading.Thread(
                target=self._keyboard_listener, args=(stop_event,), daemon=True
            )
            kb.start()
            log.info("Controls: Space start/stop | s save | d discard | q quit")

        interval = 1.0 / self._fps
        next_tick = time.monotonic()
        try:
            while not stop_event.is_set() and self._running:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                elif now - next_tick > interval * 2:
                    log.warning("Tick overrun: %.1fms behind", (now - next_tick) * 1000)
                    next_tick = now
                self._record_tick()
                next_tick += interval
                if deadline is not None and time.monotonic() >= deadline:
                    self.stop_recording()
                    self.save_episode()
                    break
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._running = False
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
        if self._recording:
            try:
                self.stop_recording()
            except Exception:
                pass
        try:
            self._cameras.stop_all()
        except Exception as exc:
            log.warning("Camera stop error: %s", exc)
        if self._ros_hub is not None:
            try:
                self._ros_hub.stop()
            except Exception as exc:
                log.warning("ROS2 hub stop error: %s", exc)
        if self._arm is not None:
            try:
                self._arm.stop()
            except Exception as exc:
                log.warning("arm adapter stop error: %s", exc)
        try:
            self._writer.finalize()
        except Exception as exc:
            log.warning("Writer finalize error: %s", exc)
        if self._ros is not None:
            try:
                self._ros.terminate()
            except Exception:
                pass
        log.info("Collector shut down")

    def _keyboard_listener(self, stop_event: threading.Event) -> None:
        import select
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not stop_event.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                else:
                    continue
                if ch == " ":
                    if not self._recording:
                        self.start_recording()
                    else:
                        self.stop_recording()
                elif ch in ("\r", "\n", "s"):
                    self.save_episode()
                elif ch in ("d", "\x7f"):
                    self.discard_episode()
                elif ch in ("q", "\x03"):
                    stop_event.set()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
