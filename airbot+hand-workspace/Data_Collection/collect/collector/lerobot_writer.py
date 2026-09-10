"""LeRobot v3 writer with a JSONL fallback when vendor runtime deps are missing."""

from __future__ import annotations

import json
import logging
import shutil
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from collector.features import build_features, compose_action
from collector.paths import VENDOR_LEROBOT_SRC, ensure_imports

log = logging.getLogger(__name__)


def _to_jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        if value.dtype == np.uint8 and value.ndim == 3:
            return {"dtype": "uint8", "shape": list(value.shape), "mean": float(value.mean())}
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    return value


def _is_jsonl_stub(dataset_path: Path) -> bool:
    info_path = dataset_path / "meta" / "info.json"
    if not info_path.is_file():
        return False
    try:
        info = json.loads(info_path.read_text(encoding="utf-8"))
    except Exception:
        return False
    return "jsonl" in str(info.get("codebase_version", "")).lower()


class LeRobotWriter:
    """High-level wrapper. Uses vendor LeRobotDataset when importable, else JSONL."""

    def __init__(
        self,
        repo_id: str,
        root: str | Path,
        fps: int = 10,
        use_videos: bool = True,
        robot_type: str = "airbot",
        task: str = "pick_cup",
        camera_names: list[str] | tuple[str, ...] | None = None,
        hand_side: str = "right",
    ):
        self._repo_id = repo_id
        self._root = Path(root)
        self._fps = fps
        self._use_videos = use_videos
        self._robot_type = robot_type
        self._task = task
        self._features = build_features(camera_names=camera_names, hand_side=hand_side)
        self._dataset = None
        self._backend = "jsonl"
        self._frame_count = 0
        self._episode_count = 0
        self._pending: list[dict] = []
        self._episode_dir: Optional[Path] = None

    def initialize(self) -> None:
        ensure_imports()
        dataset_path = self._root / self._repo_id
        self._root.mkdir(parents=True, exist_ok=True)
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset

            log.info("Using vendor LeRobot at %s", VENDOR_LEROBOT_SRC)
            self._dataset = self._open_lerobot_dataset(LeRobotDataset, dataset_path)
            self._backend = "lerobot"
            self._episode_count = int(self._dataset.meta.total_episodes)
        except Exception as exc:
            log.warning(
                "Vendor LeRobot not usable (%s). "
                "Install Data_Collection/collect/requirements-lerobot-record.txt "
                "in the collection container. Falling back to JSONL under %s",
                exc,
                dataset_path,
            )
            self._init_jsonl_fallback(dataset_path)
        log.info("Writer backend=%s episodes=%d path=%s", self._backend, self._episode_count, dataset_path)

    def _open_lerobot_dataset(self, cls: Any, dataset_path: Path) -> Any:
        if _is_jsonl_stub(dataset_path):
            bak = dataset_path.with_name(dataset_path.name + ".jsonl_stub")
            if bak.exists():
                shutil.rmtree(bak)
            log.warning("Moving JSONL stub dataset aside → %s", bak)
            dataset_path.rename(bak)
        info_path = dataset_path / "meta" / "info.json"
        if info_path.is_file() and not _is_jsonl_stub(dataset_path):
            log.info("Resuming LeRobot dataset at %s", dataset_path)
            return cls.resume(repo_id=self._repo_id, root=str(dataset_path))
        if dataset_path.exists() and not any(dataset_path.iterdir()):
            dataset_path.rmdir()
        log.info("Creating LeRobot dataset %s", dataset_path)
        return cls.create(
            repo_id=self._repo_id,
            fps=self._fps,
            features=self._features,
            root=str(dataset_path),
            robot_type=self._robot_type,
            use_videos=self._use_videos,
            image_writer_threads=4,
            batch_encoding_size=1,
        )

    def _init_jsonl_fallback(self, dataset_path: Path) -> None:
        self._backend = "jsonl"
        self._dataset = None
        dataset_path.mkdir(parents=True, exist_ok=True)
        meta_dir = dataset_path / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)
        info = {
            "codebase_version": "v3.0-jsonl-fallback",
            "robot_type": self._robot_type,
            "fps": self._fps,
            "features": self._features,
            "task": self._task,
        }
        (meta_dir / "info.json").write_text(json.dumps(info, indent=2) + "\n")
        existing = (
            sorted((dataset_path / "episodes").glob("episode-*"))
            if (dataset_path / "episodes").exists()
            else []
        )
        self._episode_count = len(existing)

    def add_frame(self, frame_data: dict[str, Any]) -> None:
        if "task" not in frame_data:
            frame_data["task"] = self._task
        if "action" not in frame_data:
            frame_data["action"] = compose_action(frame_data)
        if self._backend == "lerobot":
            allowed = set(self._features) | {"task"}
            frame_data = {k: v for k, v in frame_data.items() if k in allowed}
            self._dataset.add_frame(frame_data)
        else:
            self._pending.append(frame_data)
        self._frame_count += 1

    def save_episode(self) -> None:
        if self._backend == "lerobot":
            if self._dataset is None:
                return
            if not self._dataset.has_pending_frames():
                log.warning("save_episode called but no frames buffered")
                return
            t0 = time.time()
            self._dataset.save_episode()
            log.info("Episode saved via LeRobot in %.1fs", time.time() - t0)
            self._episode_count += 1
            self._cleanup_empty_images_dir()
            return
        if not self._pending:
            log.warning("save_episode called but no frames buffered")
            return
        ep_idx = self._episode_count
        ep_dir = self._root / self._repo_id / "episodes" / f"episode-{ep_idx:06d}"
        ep_dir.mkdir(parents=True, exist_ok=True)
        path = ep_dir / "frames.jsonl"
        with path.open("w") as handle:
            for frame in self._pending:
                row = {k: _to_jsonable(v) for k, v in frame.items()}
                handle.write(json.dumps(row) + "\n")
        (ep_dir / "episode.json").write_text(
            json.dumps(
                {
                    "episode_index": ep_idx,
                    "num_frames": len(self._pending),
                    "fps": self._fps,
                    "task": self._task,
                },
                indent=2,
            )
            + "\n"
        )
        log.info("Episode %d saved JSONL (%d frames) → %s", ep_idx, len(self._pending), path)
        self._pending = []
        self._episode_count += 1
        self._frame_count = 0

    def discard_episode(self) -> None:
        if self._backend == "lerobot" and self._dataset is not None:
            ep_idx = self._dataset.meta.total_episodes
            self._dataset.clear_episode_buffer()
            for cam_key in getattr(self._dataset.meta, "video_keys", []):
                img_dir = self._dataset.root / "images" / cam_key / f"episode-{ep_idx:06d}"
                if img_dir.is_dir():
                    shutil.rmtree(img_dir, ignore_errors=True)
        self._pending = []
        self._frame_count = 0
        log.info("Episode discarded")

    def finalize(self) -> None:
        if self._backend == "lerobot" and self._dataset is not None:
            try:
                self._dataset.finalize()
            except Exception as exc:
                log.warning("Dataset finalize error: %s", exc)
            self._cleanup_empty_images_dir()

    def _cleanup_empty_images_dir(self) -> None:
        images_dir = self._root / self._repo_id / "images"
        if images_dir.exists():
            shutil.rmtree(images_dir, ignore_errors=True)

    @property
    def episode_count(self) -> int:
        return self._episode_count

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def backend(self) -> str:
        return self._backend

    @property
    def dataset_path(self) -> Path:
        return self._root / self._repo_id
