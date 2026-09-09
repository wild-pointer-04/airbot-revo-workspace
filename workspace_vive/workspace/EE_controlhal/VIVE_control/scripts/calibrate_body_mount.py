#!/usr/bin/env python3
"""Calibrate the waist Tracker's fixed tracker-to-body mounting rotation.

This program only starts the VIVE tracker backend and reads poses.  It never
constructs a teleoperation pipeline or sends a robot command.  With the waist
tracker left fixed on the operator, move the hand tracker by known body axes:
forward, left, and up.  The observed world directions, together with the
waist tracker's orientation, solve the fixed matrix E used by HeadBase:

    body_vector = E @ tracker_local_vector

Run this before calibrate_mapping.py whenever the waist mount changes.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
for _path in (str(ROOT), str(MIDDLE)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from packet_source import _wxyz_to_xyzw  # noqa: E402
from processing.coordinate_transform import rotation_matrix_from_values  # noqa: E402
from processing.head_base import HeadBase
from processing.math_pose import matrix_from_quat, quat_from_matrix
from processing.vr_normalize import to_teleop_pose  # noqa: E402
from protocol.pico_packet import Pose  # noqa: E402
from vive_tracking.tracker_manager import TrackerManager, create_backend, load_config  # noqa: E402

SETTLE_S = 2.0
SAMPLE_DT = 0.02
MIN_TRAVEL_M = 0.12
# The requested moves are body +X/+Y/+Z, so their normalised observations must
# be close to an orthogonal, right-handed frame before a mount is written.
MIN_AXIS_ALIGNMENT = 0.90
MAX_AXIS_DOT = 0.35
MIN_HANDEDNESS = 0.60
MAX_BODY_ROTATION_RAD = np.deg2rad(12.0)
MIN_RUNTIME_ALIGNMENT = float(np.cos(np.deg2rad(15.0)))
MIN_SAMPLE_COUNT = 10
MAX_POSE_AGE_S = 0.15
MAX_PAIR_SKEW_S = 0.05
MAX_POSITION_JITTER_M = 0.02
MIN_SAMPLE_COVERAGE = 0.80
SAMPLE_QUALITY_FIELDS = (
    "position_jitter_m",
    "valid_fraction",
    "sample_span_s",
    "sample_duration_s",
)
AXES = ("forward", "left", "up")
STEPS = (
    (
        "baseline",
        "站直并保持腰部 tracker 的姿势，手腕 tracker 放在身体前方的起始位置",
    ),
    (
        "forward",
        "腰部不动，把手腕 tracker 沿身体正前方平移至少 30 cm",
    ),
    (
        "left",
        "回到起始位置后，腰部不动，把手腕 tracker 沿身体左方平移至少 30 cm",
    ),
    (
        "up",
        "回到起始位置后，腰部不动，把手腕 tracker 竖直向上平移至少 30 cm",
    ),
)


def _pose(tracker: Any) -> Pose:
    return Pose(
        position=np.asarray(tracker.position_m.as_list(), dtype=np.float64),
        rotation=_wxyz_to_xyzw(tracker.orientation),
    )


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    """Return the closest proper rotation to a collection of pose matrices."""
    if not rotations:
        raise RuntimeError("没有有效的腰部 tracker 姿态")
    u, _singular_values, vh = np.linalg.svd(np.mean(rotations, axis=0))
    result = u @ vh
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vh
    return result


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))


def _fresh_pair(manager: TrackerManager, hand_name: str, body_name: str):
    hand = manager.get_pose(hand_name)
    body = manager.get_pose(body_name)
    now_ns = time.monotonic_ns()
    for tracker in (hand, body):
        if tracker is None or not tracker.tracking_valid:
            return None
        age_s = (now_ns - tracker.host_monotonic_ns) / 1e9
        if tracker.host_monotonic_ns <= 0 or not 0.0 <= age_s <= MAX_POSE_AGE_S:
            return None
        position = np.asarray(tracker.position_m.as_list(), dtype=np.float64)
        orientation = np.asarray(tracker.orientation.as_wxyz(), dtype=np.float64)
        if (
            not np.isfinite(position).all()
            or not np.isfinite(orientation).all()
            or np.linalg.norm(orientation) < 1e-9
        ):
            return None
    if abs(hand.host_monotonic_ns - body.host_monotonic_ns) / 1e9 > MAX_PAIR_SKEW_S:
        return None
    return hand, body


def _wait_for_trackers(
    manager: TrackerManager, hand_name: str, body_name: str, *, timeout: float = 20.0
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _fresh_pair(manager, hand_name, body_name) is not None:
            return
        time.sleep(SAMPLE_DT)
    raise RuntimeError(f"超时：{hand_name} 和 {body_name} 未同时提供有效新位姿")


def _sample_quality_ok(sample: dict[str, Any]) -> bool:
    if "count" in sample and int(sample["count"]) < MIN_SAMPLE_COUNT:
        return False
    if "position_jitter_m" in sample and not (
        0.0 <= float(sample["position_jitter_m"]) <= MAX_POSITION_JITTER_M
    ):
        return False
    if "valid_fraction" in sample and not (
        MIN_SAMPLE_COVERAGE <= float(sample["valid_fraction"]) <= 1.0
    ):
        return False
    if "sample_span_s" in sample or "sample_duration_s" in sample:
        duration = float(sample.get("sample_duration_s", 0.0))
        span = float(sample.get("sample_span_s", 0.0))
        if not np.isfinite(duration) or duration <= 0.0 or not (
            MIN_SAMPLE_COVERAGE * duration <= span <= duration + SAMPLE_DT
        ):
            return False
    return True


def _sample(
    manager: TrackerManager,
    hand_name: str,
    body_name: str,
    vr_space: str,
    *,
    settle_s: float = SETTLE_S,
) -> dict[str, Any]:
    """Average hand-minus-waist vectors and waist orientation while stationary."""
    if not np.isfinite(settle_s) or settle_s <= 0.0:
        raise ValueError("采样时间必须为有限正数")
    _wait_for_trackers(manager, hand_name, body_name)
    relative_positions: list[np.ndarray] = []
    hand_positions: list[np.ndarray] = []
    body_positions: list[np.ndarray] = []
    body_rotations: list[np.ndarray] = []
    sample_times: list[float] = []
    hand_sequences: set[int] = set()
    body_sequences: set[int] = set()
    attempts = 0
    fresh_attempts = 0
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        time.sleep(SAMPLE_DT)
        attempts += 1
        pair = _fresh_pair(manager, hand_name, body_name)
        if pair is None:
            continue
        fresh_attempts += 1
        hand, body = pair
        if hand.sequence_id in hand_sequences or body.sequence_id in body_sequences:
            continue
        hand_sequences.add(hand.sequence_id)
        body_sequences.add(body.sequence_id)
        hand_t = to_teleop_pose(_pose(hand), vr_space)
        body_t = to_teleop_pose(_pose(body), vr_space)
        relative_positions.append(hand_t.position - body_t.position)
        hand_positions.append(hand_t.position)
        body_positions.append(body_t.position)
        body_rotations.append(matrix_from_quat(body_t.rotation))
        sample_times.append(time.monotonic())

    if len(relative_positions) < MIN_SAMPLE_COUNT:
        raise RuntimeError(
            f"独立双 tracker 新帧不足：{len(relative_positions)} < {MIN_SAMPLE_COUNT}"
        )

    body_rotation = _mean_rotation(body_rotations)
    max_body_jitter = max(
        _rotation_angle(body_rotation.T @ rotation) for rotation in body_rotations
    )
    position_jitter = 0.0
    for positions in (relative_positions, hand_positions, body_positions):
        centered = np.asarray(positions) - np.mean(positions, axis=0)
        position_jitter = max(
            position_jitter, float(np.max(np.linalg.norm(centered, axis=1)))
        )
    sample = {
        "count": len(relative_positions),
        "relative_position": np.mean(relative_positions, axis=0),
        "body_rotation": body_rotation,
        "body_jitter_rad": max_body_jitter,
        "position_jitter_m": position_jitter,
        "valid_fraction": fresh_attempts / attempts,
        "sample_span_s": sample_times[-1] - sample_times[0],
        "sample_duration_s": settle_s,
    }
    if not _sample_quality_ok(sample) or max_body_jitter > MAX_BODY_ROTATION_RAD:
        raise RuntimeError("采样不合格：位置未停稳、腰部转动或有效新帧覆盖不足，请重采")
    return sample


def _write_sample(path: Path, key: str, sample: dict[str, Any]) -> None:
    """Persist one stationary sample for the non-interactive calibration flow."""
    raw = (
        yaml.safe_load(path.read_text(encoding="utf-8"))
        if key != "baseline" and path.is_file()
        else {}
    )
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise RuntimeError(f"{path} 必须是 YAML 映射")
    raw[key] = {
        "count": int(sample["count"]),
        "relative_position": np.asarray(sample["relative_position"]).tolist(),
        "body_rotation": np.asarray(sample["body_rotation"]).tolist(),
        "body_jitter_rad": float(sample["body_jitter_rad"]),
    }
    for field in SAMPLE_QUALITY_FIELDS:
        if field in sample:
            raw[key][field] = float(sample[field])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


def _read_samples(path: Path) -> dict[str, dict[str, Any]]:
    if not path.is_file():
        raise RuntimeError(f"找不到采样文件：{path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise RuntimeError(f"{path} 必须是 YAML 映射")
    required = ("baseline", *AXES)
    missing = [key for key in required if key not in raw]
    if missing:
        raise RuntimeError(f"采样文件缺少步骤：{', '.join(missing)}")
    recorded: dict[str, dict[str, Any]] = {}
    for key in required:
        sample = raw[key]
        if not isinstance(sample, dict):
            raise RuntimeError(f"采样 {key} 格式无效")
        try:
            recorded[key] = {
                "count": int(sample["count"]),
                "relative_position": np.asarray(
                    sample["relative_position"], dtype=np.float64
                ),
                "body_rotation": np.asarray(sample["body_rotation"], dtype=np.float64),
                "body_jitter_rad": float(sample["body_jitter_rad"]),
            }
            for field in SAMPLE_QUALITY_FIELDS:
                if field in sample:
                    recorded[key][field] = float(sample[field])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"采样 {key} 格式无效：{exc}") from exc
    return recorded


def _measure(manager: TrackerManager, config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    vive = dict(config.get("vive") or {})
    transform = dict(config.get("transform") or {})
    hand_name = str(vive.get("hand", "hand_controller"))
    body_name = str(vive.get("head", "body_hmd"))
    vr_space = str(transform.get("vr_space", "unity"))

    print(f"等待 {body_name}（腰部）和 {hand_name}（手腕）出位姿…")
    _wait_for_trackers(manager, hand_name, body_name)

    print("\n腰部 Tracker 安装标定：此程序不会控制机械臂。")
    print("每一步按回车后采样 2 秒。腰部 tracker 必须全程绑紧、身体朝向不变。")
    recorded: dict[str, dict[str, Any]] = {}
    for key, prompt in STEPS:
        input(f"\n>>> {prompt}；摆好后按回车：")
        print(f"    采样 {SETTLE_S:.0f} 秒，请保持不动…")
        recorded[key] = _sample(manager, hand_name, body_name, vr_space)
        rel = recorded[key]["relative_position"]
        jitter_deg = np.rad2deg(recorded[key]["body_jitter_rad"])
        print(
            f"    hand-minus-waist = [{rel[0]:+.3f}, {rel[1]:+.3f}, {rel[2]:+.3f}] m"
            f"；腰部抖动 {jitter_deg:.1f} deg"
        )
    return recorded


def _nearest_rotation(columns: np.ndarray) -> np.ndarray:
    u, _singular_values, vh = np.linalg.svd(columns)
    result = u @ vh
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vh
    return result


def _solve(recorded: dict[str, dict[str, Any]]) -> tuple[np.ndarray, dict[str, Any]]:
    """Solve E from body-axis hand displacements and a waist tracker pose.

    Let Rwt map tracker-local vectors to the tracking world and Rwb map body
    vectors to the tracking world.  The hand displacements directly estimate
    Rwb's columns.  Since Rwb = Rwt @ E.T, E = Rwb.T @ Rwt.
    """
    baseline = recorded["baseline"]
    baseline_relative = np.asarray(baseline["relative_position"], dtype=np.float64)
    r_world_tracker = rotation_matrix_from_values(
        baseline["body_rotation"], name="baseline body tracker rotation"
    )

    deltas: dict[str, np.ndarray] = {}
    travel: dict[str, float] = {}
    for axis in AXES:
        delta = (
            np.asarray(recorded[axis]["relative_position"], dtype=np.float64)
            - baseline_relative
        )
        distance = float(np.linalg.norm(delta))
        if distance <= np.finfo(np.float64).eps:
            raise ValueError(f"{axis} 位移为零，无法求解安装方向")
        deltas[axis] = delta
        travel[axis] = distance

    observed_columns = np.column_stack([deltas[axis] / travel[axis] for axis in AXES])
    raw_handedness = float(np.linalg.det(observed_columns))
    r_world_body = _nearest_rotation(observed_columns)
    tracker_to_body = rotation_matrix_from_values(
        r_world_body.T @ r_world_tracker, name="solved head_tracker_to_body"
    )

    pairwise = {
        f"{first}_{second}": float(
            np.dot(observed_columns[:, first_index], observed_columns[:, second_index])
        )
        for first_index, first in enumerate(AXES)
        for second_index, second in enumerate(AXES)
        if first_index < second_index
    }
    body_turn = {
        axis: _rotation_angle(
            r_world_tracker.T
            @ rotation_matrix_from_values(
                recorded[axis]["body_rotation"], name=f"{axis} body tracker rotation"
            )
        )
        for axis in AXES
    }
    alignment = {
        axis: float(np.dot(observed_columns[:, index], r_world_body[:, index]))
        for index, axis in enumerate(AXES)
    }
    jitter = {
        key: float(sample.get("body_jitter_rad", 0.0)) for key, sample in recorded.items()
    }
    world_up_alignment = float(observed_columns[2, 2])
    head_base = HeadBase(tracker_to_body)
    runtime_positions = {}
    for key in ("baseline", *AXES):
        sample = recorded[key]
        body = Pose(np.zeros(3), quat_from_matrix(sample["body_rotation"]))
        hand = Pose(
            np.asarray(sample["relative_position"]), np.array([0.0, 0.0, 0.0, 1.0])
        )
        runtime_positions[key] = head_base.relative(body, True, hand).position
    runtime_deltas = {
        axis: runtime_positions[axis] - runtime_positions["baseline"] for axis in AXES
    }
    runtime_alignment = {}
    for index, axis in enumerate(AXES):
        delta = runtime_deltas[axis]
        distance = float(np.linalg.norm(delta))
        runtime_alignment[axis] = float(delta[index] / distance) if distance > 0.0 else 0.0
    checks = {
        "travel": all(distance >= MIN_TRAVEL_M for distance in travel.values()),
        "orthogonal": all(abs(value) <= MAX_AXIS_DOT for value in pairwise.values()),
        "right_handed": raw_handedness >= MIN_HANDEDNESS,
        "axis_alignment": all(value >= MIN_AXIS_ALIGNMENT for value in alignment.values()),
        "body_still": all(value <= MAX_BODY_ROTATION_RAD for value in body_turn.values()),
        "sample_stable": all(value <= MAX_BODY_ROTATION_RAD for value in jitter.values()),
        "sample_quality": all(_sample_quality_ok(sample) for sample in recorded.values()),
        "world_up": world_up_alignment >= MIN_RUNTIME_ALIGNMENT,
        "runtime_axes": all(
            runtime_alignment[axis] >= MIN_RUNTIME_ALIGNMENT
            and np.linalg.norm(runtime_deltas[axis]) >= MIN_TRAVEL_M
            for axis in AXES
        ),
    }
    stats: dict[str, Any] = {
        "deltas": deltas,
        "travel": travel,
        "observed_columns": observed_columns,
        "raw_handedness": raw_handedness,
        "pairwise": pairwise,
        "body_turn": body_turn,
        "alignment": alignment,
        "jitter": jitter,
        "world_up_alignment": world_up_alignment,
        "runtime_deltas": runtime_deltas,
        "runtime_alignment": runtime_alignment,
        "checks": checks,
        "ok": all(checks.values()),
    }
    return tracker_to_body, stats


def _rotation_block(matrix: np.ndarray) -> list[str]:
    rotation = rotation_matrix_from_values(matrix, name="head_tracker_to_body")
    lines = ["  head_tracker_to_body:\n", "    rotation_matrix:\n"]
    for row in rotation:
        values = ", ".join(f"{float(value):.10f}" for value in row)
        lines.append(f"      - [{values}]\n")
    return lines


def _write_config(config_path: Path, matrix: np.ndarray) -> None:
    """Replace only teleop.head_tracker_to_body while preserving other YAML text."""
    lines = config_path.read_text(encoding="utf-8").splitlines(keepends=True)
    teleop_indent: int | None = None
    for index, line in enumerate(lines):
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if stripped == "teleop:":
            teleop_indent = indent
            continue
        if teleop_indent is None:
            continue
        if stripped and indent <= teleop_indent:
            break
        if stripped != "head_tracker_to_body:":
            continue
        end = index + 1
        while end < len(lines):
            candidate = lines[end]
            stripped = candidate.strip()
            if stripped and len(candidate) - len(candidate.lstrip()) <= indent:
                break
            end += 1
        replacement = _rotation_block(matrix)
        replacement[0] = " " * indent + replacement[0].lstrip()
        child_indent = " " * (indent + 2)
        replacement[1] = child_indent + replacement[1].lstrip()
        replacement[2:] = [
            " " * (indent + 4) + value.lstrip() for value in replacement[2:]
        ]
        config_path.write_text(
            "".join(lines[:index] + replacement + lines[end:]), encoding="utf-8"
        )
        return
    raise RuntimeError(f"{config_path} 中找不到 teleop.head_tracker_to_body")


def _resolve_tracker_config(vive_cfg: dict[str, Any]) -> Path:
    path = Path(vive_cfg.get("tracker_config") or "config/tracker.yaml")
    return path if path.is_absolute() else ROOT / path


def _report(matrix: np.ndarray, stats: dict[str, Any]) -> None:
    print("\n" + "=" * 72)
    print("求得的 body = E @ tracker 安装矩阵 E")
    print("=" * 72)
    print(np.array2string(matrix, precision=5, suppress_small=True))
    print(f"det(E) = {np.linalg.det(matrix):+.6f}")
    print(f"实测上方向与世界 +Z 对齐 = {stats['world_up_alignment']:.3f}")
    print("\n动作观测（tracking/teleop 世界坐标）：")
    for axis in AXES:
        delta = stats["deltas"][axis]
        print(
            f"  {axis:7s} Δ=[{delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f}] m"
            f"  行程={stats['travel'][axis] * 100:.1f} cm"
            f"  对齐={stats['alignment'][axis]:.3f}"
            f"  腰部转动={np.rad2deg(stats['body_turn'][axis]):.1f} deg"
        )
        print(
            f"    HeadBase 实际增量 = {np.array2string(stats['runtime_deltas'][axis], precision=3)}"
            f"  对齐={stats['runtime_alignment'][axis]:.3f}"
        )
    print(f"右手性 det([forward,left,up]) = {stats['raw_handedness']:+.3f}")
    print(
        "轴间点积："
        + ", ".join(f" {key}={value:+.3f}" for key, value in stats["pairwise"].items())
    )
    for name, passed in stats["checks"].items():
        print(f"  {'✓' if passed else '✗'} {name}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="标定腰部 VIVE tracker 的安装旋转（不控制机器人）"
    )
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "teleop.yaml")
    parser.add_argument(
        "--write",
        action="store_true",
        help="仅在全部质量检查通过时写入 teleop.head_tracker_to_body",
    )
    batch_mode = parser.add_mutually_exclusive_group()
    batch_mode.add_argument(
        "--sample",
        choices=("baseline", *AXES),
        help="采集一个已摆好的静止姿势后退出，不读取 stdin",
    )
    batch_mode.add_argument(
        "--solve",
        action="store_true",
        help="从 --samples 读取四个已采集姿势并求解安装外参",
    )
    parser.add_argument(
        "--samples",
        type=Path,
        default=Path("/tmp/vive_body_mount_samples.yaml"),
        help="单步采样的临时 YAML 文件（默认：/tmp/vive_body_mount_samples.yaml）",
    )
    parser.add_argument(
        "--sample-seconds",
        type=float,
        default=SETTLE_S,
        help=f"单步采样持续秒数（默认：{SETTLE_S:.0f}）",
    )
    args = parser.parse_args()
    if not np.isfinite(args.sample_seconds) or args.sample_seconds <= 0.0:
        parser.error("--sample-seconds 必须大于 0")
    config_path = args.config
    if not config_path.is_file():
        print(f"错误：{config_path} 不存在")
        return 1
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if args.solve:
        try:
            recorded = _read_samples(args.samples)
            matrix, stats = _solve(recorded)
        except (RuntimeError, ValueError) as exc:
            print(f"标定失败：{exc}")
            return 1
        _report(matrix, stats)
        if not stats["ok"]:
            print("\n结果未写入。重新采集四个动作，保持腰部 tracker 姿势不变。")
            return 1
        if args.write:
            _write_config(config_path, matrix)
            print(f"\n已只更新 {config_path} 中的 teleop.head_tracker_to_body。")
        else:
            print("\n验证通过，未写入。传 --write 才会更新 teleop.head_tracker_to_body。")
        return 0

    tracker_config = _resolve_tracker_config(dict(config.get("vive") or {}))
    if not tracker_config.is_file():
        print(f"错误：tracker config {tracker_config} 不存在")
        return 1

    # Body-mount samples require a solved world pose.  The teleop config keeps
    # lighthouse geometry frozen, so opt into the separate calibration mode
    # only for this reader instance.  No robot pipeline is constructed here.
    tracker_runtime = load_config(tracker_config)
    libsurvive_cfg = dict(tracker_runtime.get("libsurvive") or {})
    libsurvive_cfg["lighthouse_calibration"] = True
    tracker_runtime["libsurvive"] = libsurvive_cfg
    manager = TrackerManager(create_backend(tracker_runtime), tracker_runtime)
    manager.start()
    try:
        if args.sample:
            vive = dict(config.get("vive") or {})
            hand_name = str(vive.get("hand", "hand_controller"))
            body_name = str(vive.get("head", "body_hmd"))
            vr_space = str((config.get("transform") or {}).get("vr_space", "unity"))
            print(f"等待 {body_name}（腰部）和 {hand_name}（手腕）出位姿…")
            _wait_for_trackers(manager, hand_name, body_name)
            print(f"采样 {args.sample} {args.sample_seconds:.1f} 秒，请保持不动…")
            sample = _sample(
                manager,
                hand_name,
                body_name,
                vr_space,
                settle_s=args.sample_seconds,
            )
            _write_sample(args.samples, args.sample, sample)
            rel = sample["relative_position"]
            print(
                f"已保存 {args.sample}: [{rel[0]:+.3f}, {rel[1]:+.3f}, {rel[2]:+.3f}] m"
                f"，腰部抖动 {np.rad2deg(sample['body_jitter_rad']):.1f} deg"
            )
            return 0
        recorded = _measure(manager, config)
        matrix, stats = _solve(recorded)
    except (RuntimeError, ValueError) as exc:
        print(f"标定失败：{exc}")
        return 1
    except KeyboardInterrupt:
        print("\n标定已取消，未写入配置。")
        return 130
    finally:
        manager.stop()

    _report(matrix, stats)
    if not stats["ok"]:
        print(
            "\n结果未写入。回到起始位置后重跑，保持腰部 tracker 姿势不变，"
            "并做更长、更单轴的手部位移。"
        )
        return 1
    if args.write:
        _write_config(config_path, matrix)
        print(f"\n已只更新 {config_path} 中的 teleop.head_tracker_to_body。")
    else:
        print("\n验证通过，未写入。传 --write 才会更新 teleop.head_tracker_to_body。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
