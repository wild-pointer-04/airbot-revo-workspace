"""JSONL recorder: two VIVE trackers + pipeline command + actual arm EE."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, TextIO

import numpy as np

from processing.vr_normalize import to_teleop_pose


def _xyz(vec) -> list[float]:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    return [float(arr[0]), float(arr[1]), float(arr[2])]


def _xyzw(quat) -> list[float]:
    arr = np.asarray(quat, dtype=np.float64).reshape(-1)
    return [float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])]


class MappingRecorder:
    def __init__(self, path: Path, *, meta: dict[str, Any], period_s: float = 0.02) -> None:
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._period = float(period_s)
        self._last = 0.0
        self._count = 0
        self._handle: TextIO = self._path.open("w", encoding="utf-8")
        self._handle.write(json.dumps({"type": "meta", **meta}, ensure_ascii=False) + "\n")
        self._handle.flush()

    @property
    def path(self) -> Path:
        return self._path

    @property
    def count(self) -> int:
        return self._count

    def maybe_write(self, *, pipeline, packet, result, ee_pose) -> None:
        now = time.monotonic()
        if self._last > 0.0 and (now - self._last) < self._period:
            return
        self._last = now
        vr = pipeline.vr_space
        head_t = to_teleop_pose(packet.hmd.pose, vr)
        hand_t = to_teleop_pose(packet.right_controller.pose, vr)
        hand_rel = pipeline.head_base.relative(head_t, bool(packet.hmd.valid), hand_t)
        cmd = result.command
        delta = result.delta_pos
        row = {
            "type": "sample",
            "t": now,
            "seq": int(packet.seq),
            "state": result.state.value,
            "clutch": bool(result.clutch_active),
            "head_valid": bool(packet.hmd.valid),
            "hand_valid": bool(packet.right_controller.valid),
            "vive_head_raw": _xyz(packet.hmd.position),
            "vive_hand_raw": _xyz(packet.right_controller.position),
            "vive_head_teleop": _xyz(head_t.position),
            "vive_hand_teleop": _xyz(hand_t.position),
            "vive_hand_rel": _xyz(hand_rel.position),
            "cmd": None if cmd is None else _xyz(cmd.position),
            "delta": None if delta is None else _xyz(delta),
            "arm_ee": _xyz(ee_pose.position),
            "arm_ee_rot": _xyzw(ee_pose.rotation),
        }
        self._handle.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._count += 1
        if self._count % 25 == 0:
            self._handle.flush()

    def close(self) -> None:
        self._handle.flush()
        self._handle.close()
