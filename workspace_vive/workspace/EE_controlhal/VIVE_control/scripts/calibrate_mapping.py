#!/usr/bin/env python3
"""Measure the tracker→AIRBOT SDK mapping end-to-end and solve axis_map.

A delta leaving the hand tracker passes through vr_normalize (device frame →
teleop_standard) and head_base (yaw-only operator frame, including the
head_tracker_to_body mounting calibration) before axis_map ever sees it. Reasoning about those three
stages by hand is what produced the 09-02/09-04 mis-mappings, so this script
samples the pipeline's own output for three physical motions and solves for the
transform that makes the AIRBOT move physically up/forward/left.

Run it in the container that actually runs the teleop:
  docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
    airbot-revo-humble \
    python3 scripts/calibrate_mapping.py --write
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
for _p in (str(ROOT), str(MIDDLE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from packet_source import _wxyz_to_xyzw  # noqa: E402
from processing.head_base import HeadBase  # noqa: E402
from processing.vr_normalize import to_teleop_pose  # noqa: E402
from protocol.pico_packet import Pose  # noqa: E402
from vive_tracking.tracker_manager import TrackerManager  # noqa: E402

SETTLE_S = 2.0
SAMPLE_DT = 0.02
# Below this the motion is indistinguishable from tracker noise.
MIN_TRAVEL_M = 0.12
# A clean signed permutation has one dominant component per axis.
DOMINANT = 0.80
RESIDUAL = 0.45
# Head must stay put: its pose defines the frame every delta is measured in.
MAX_HEAD_DRIFT_M = 0.08

STEPS = (
    ("baseline", "站好，手臂自然前伸，tracker 停在起始位置"),
    ("up", "把 tracker 竖直举高约 30cm（只往上，不要前后左右）"),
    ("forward", "回到起始高度，朝机器人方向平移约 30cm（只往前）"),
    ("left", "回到起始位置，朝你的左手边平移约 30cm（只往左）"),
)

# Motion → AIRBOT SDK command axis. This SDK uses +X down, +Y left, +Z
# forward, while teleop_standard uses +X forward, +Y left, +Z up.
TARGET = {
    "up": (0, -1.0, "机械臂物理向上 (SDK -X)"),
    "forward": (2, +1.0, "机械臂物理向前 (SDK +Z)"),
    "left": (1, +1.0, "机械臂物理向左 (SDK +Y)"),
}
AXIS_NAME = ("x", "y", "z")


class Rig:
    """Replays the teleop pipeline's pose path without touching the robot.

    pipeline._hand_rel does vr_normalize → head_base → filter. The filter only
    smooths, so it is left out; everything before axis_map is reproduced here so
    the solve sees exactly what Clutch would hand to CoordinateTransform.
    """

    def __init__(self, teleop_cfg: dict, transform_cfg: dict) -> None:
        self.vr_space = str(transform_cfg.get("vr_space", "unity"))
        self.head_base_enabled = bool(teleop_cfg.get("head_base", True))
        # Match TeleopPipeline exactly so calibration sees the same body yaw
        # frame as the running teleoperation process.
        self.head_base = HeadBase.from_config(teleop_cfg)

    def hand_rel(self, hand, head) -> np.ndarray:
        hand_t = to_teleop_pose(_pose(hand), self.vr_space)
        if not self.head_base_enabled:
            return np.asarray(hand_t.position, dtype=np.float64)
        head_valid = head is not None and head.tracking_valid
        head_t = to_teleop_pose(_pose(head), self.vr_space) if head is not None else None
        rel = self.head_base.relative(head_t, bool(head_valid), hand_t)
        return np.asarray(rel.position, dtype=np.float64)


def _pose(tracker) -> Pose:
    return Pose(
        position=np.array(tracker.position_m.as_list(), dtype=np.float64),
        rotation=_wxyz_to_xyzw(tracker.orientation),
    )


def _sample(manager, tracker_name, head_name, rig) -> dict:
    """window-average so jitter does not pick the axis."""
    samples_hand, samples_head = [], []
    t0 = time.time()
    while time.time() - t0 < SETTLE_S:
        time.sleep(SAMPLE_DT)
        hand, head = manager.get_pose(tracker_name), manager.get_pose(head_name)
        if hand is None or not hand.tracking_valid:
            continue
        if rig.head_base_enabled and (head is None or not head.tracking_valid):
            continue
        samples_hand.append(hand)
        samples_head.append(head)
    if not samples_hand:
        required = f"{tracker_name} + {head_name}" if rig.head_base_enabled else tracker_name
        raise RuntimeError(f"{required} 无效位姿 — 检查基站视野")
    ref = rig.hand_rel(samples_hand[-1], samples_head[-1])
    heads = np.array([np.array(h.position_m.as_list()) for h in samples_head if h and h.tracking_valid])
    head_drift = 0.0
    if len(heads) > 1:
        head_drift = float(np.linalg.norm(heads.max(0) - heads.min(0)))
    raw_all = np.array([np.array(h.position_m.as_list()) for h in samples_hand])
    teleop_all = np.array([rig.hand_rel(h, d) for h, d in zip(samples_hand, samples_head)])
    return {
        "count": len(samples_hand),
        "ref": ref.copy(),
        "raw_mean": raw_all.mean(0).copy(),
        "teleop_mean": teleop_all.mean(0).copy(),
        "head_drift": head_drift,
    }


def _measure(manager, config) -> dict:
    teleop_cfg = dict(config.get("teleop") or {})
    transform_cfg = dict(config.get("transform") or {})
    vive_cfg = dict(config.get("vive") or {})
    rig = Rig(teleop_cfg, transform_cfg)
    tracker_name = str(vive_cfg.get("hand", "hand_controller"))
    head_name = str(vive_cfg.get("head", "body_hmd"))

    print(f"\n等待 {tracker_name} + body_hmd 出位姿…")
    if manager.wait_for_pose(timeout=20.0) is None:
        raise RuntimeError("超时 — 没有位姿")

    print(f"\n当前配置:")
    print(f"  vr_space          = {rig.vr_space}")
    print(f"  head_base         = {rig.head_base_enabled}")
    print(f"  head_tracker      = {head_name}")
    print(f"  head_mount        = {teleop_cfg.get('head_tracker_to_body', 'identity')}")
    print(f"  tracker           = {tracker_name}")
    print("\n坐标系标定程序 — 按提示移动 tracker，每步停 2 秒不动")
    print("=" * 72)

    recorded: dict[str, dict] = {}
    for key, prompt in STEPS:
        input(f"\n>>> {prompt}，摆好后按回车: ")
        print(f"    采样 {SETTLE_S:.0f}s，别动…")
        recorded[key] = _sample(manager, tracker_name, head_name, rig)
        ref = recorded[key]["ref"]
        print(f"    teleop_after_head_base = [{ref[0]: .3f}, {ref[1]: .3f}, {ref[2]: .3f}]")
        if recorded[key]["head_drift"] > MAX_HEAD_DRIFT_M:
            print(
                f"    [!] 头部位移过大 ({recorded[key]['head_drift']*1000:.0f}mm > {MAX_HEAD_DRIFT_M*1000:.0f}mm)，标定结果可能偏移"
            )
    return recorded


def _solve(recorded) -> tuple[np.ndarray, dict]:
    baseline_ref = recorded["baseline"]["ref"]
    obs_delta = {}
    for key in ("up", "forward", "left"):
        obs_delta[key] = recorded[key]["ref"] - baseline_ref

    print("\n" + "=" * 72)
    print("观测增量（teleop_after_head_base 坐标）")
    print("=" * 72)
    for key in ("up", "forward", "left"):
        d = obs_delta[key]
        mag = float(np.linalg.norm(d))
        print(f"\n{key:8s}: [{d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}] m  (‖Δ‖ = {mag*100:.1f} cm)")
        idx = int(np.argmax(np.abs(d)))
        print(f"           最大分量 = 轴 {AXIS_NAME[idx]}  ({d[idx]:+.3f} m)")
        if mag < MIN_TRAVEL_M:
            print(f"           [警告] 移动距离过短 ({mag*100:.1f} cm < {MIN_TRAVEL_M*100:.0f} cm)，结果可能不准")

    up_d = obs_delta["up"]
    fwd_d = obs_delta["forward"]
    left_d = obs_delta["left"]

    # Solve per motion instead of inverting the observation matrix. R = inv(O)
    # would reproduce the unit axes exactly for ANY input, so its checks pass
    # even on garbage; and axis_map can only express a signed permutation, so a
    # non-permutation R would be silently rounded on the way to YAML. Reading
    # the dominant observed axis directly is what axis_map can actually apply,
    # and it leaves the off-axis leakage visible for the verify step.
    axis_of = {}
    for motion, delta in (("forward", fwd_d), ("left", left_d), ("up", up_d)):
        idx = int(np.argmax(np.abs(delta)))
        axis_of[motion] = (idx, float(np.sign(delta[idx])))

    R = np.zeros((3, 3), dtype=np.float64)
    for motion, (robot_axis, expected_sign, _desc) in TARGET.items():
        src_axis, src_sign = axis_of[motion]
        # Map the signed observed source component to the desired SDK axis.
        R[robot_axis, src_axis] = expected_sign * src_sign

    used = [axis_of[m][0] for m in ("forward", "left", "up")]
    # Two motions landing on one source axis means the operator moved along the
    # same physical direction twice (or noise swamped a short move): the result
    # is not a permutation and axis_map cannot represent it.
    degenerate = len(set(used)) != 3
    determinant = float(np.linalg.det(R))
    proper_rotation = bool(np.isclose(determinant, 1.0, atol=1e-6))

    stats = {}
    for motion, (expected_axis, expected_sign, _desc) in TARGET.items():
        delta = obs_delta[motion]
        travel = float(np.linalg.norm(delta))
        mapped = R @ delta if not degenerate else np.zeros(3)
        dom_idx = int(np.argmax(np.abs(mapped))) if not degenerate else -1
        dom_val = float(mapped[dom_idx]) if not degenerate else 0.0
        # Fraction of the physical motion that survives onto the intended axis.
        clean = (abs(dom_val) / travel >= DOMINANT) if travel > 0 else False
        correct_axis = dom_idx == expected_axis
        correct_sign = dom_val * expected_sign > 0
        residual_mag = (
            float(np.linalg.norm(np.delete(mapped, dom_idx))) if dom_idx >= 0 else 0.0
        )
        low_residual = residual_mag < RESIDUAL * abs(dom_val) if dom_val else False
        long_enough = travel >= MIN_TRAVEL_M
        ok = (
            not degenerate
            and clean
            and correct_axis
            and correct_sign
            and low_residual
            and long_enough
        )
        stats[motion] = {
            "clean": clean,
            "correct_axis": correct_axis,
            "correct_sign": correct_sign,
            "low_residual": low_residual,
            "long_enough": long_enough,
            "ok": ok,
            "mapped": mapped.copy(),
            "dominant_axis": dom_idx,
            "dominant_val": dom_val,
            "residual_mag": residual_mag,
            "travel": travel,
        }
    stats["_degenerate"] = degenerate
    stats["_determinant"] = determinant
    stats["_proper_rotation"] = proper_rotation
    return R, stats


def _report_solution(R, stats) -> bool:
    if stats["_degenerate"]:
        print("\n" + "=" * 72)
        print("[✗] 三次动作没有落在三个不同的轴上 — 无法求解")
        print("=" * 72)
        print("常见原因：某一步实际沿着和另一步相同的方向移动，或移动距离太短")
        print("被噪声淹没。请重跑，每步只沿一个方向移动 ≥30cm 并停稳。")
        return False

    if not stats["_proper_rotation"]:
        print("\n" + "=" * 72)
        print("[✗] 观测到反射矩阵 (det = -1)，axis_map 不能安全表达该结果")
        print("=" * 72)
        print("检查三次动作是否确实是 前 / 左 / 上，且 body tracker 在整个过程保持朝向不变。")
        print("还要检查 head_tracker_to_body；两个右手坐标系之间的变换必须满足 det = +1。")
        return False

    print("\n" + "=" * 72)
    print("建议的 axis_map 转换矩阵 R")
    print("=" * 72)
    print(np.array2string(R, precision=3, suppress_small=True))
    print(f"det(R) = {stats['_determinant']:+.0f}")

    print("\n" + "=" * 72)
    print("映射验证 (R @ 实测增量)")
    print("=" * 72)
    all_ok = True
    for motion, (_ax, _sgn, desc) in TARGET.items():
        s = stats[motion]
        mapped = s["mapped"]
        dom = s["dominant_val"]
        resid = s["residual_mag"]
        travel = s["travel"]
        print(f"\n{motion:8s} → {desc}")
        print(f"  实测行程: {travel*100:.1f} cm")
        print(f"  映射结果: [{mapped[0]:+.3f}, {mapped[1]:+.3f}, {mapped[2]:+.3f}]")
        ratio = resid / abs(dom) if dom else float("inf")
        print(
            f"  主分量 = 轴 {AXIS_NAME[s['dominant_axis']]} ({dom:+.3f})  "
            f"残差 ‖其他‖ = {resid:.3f}  ratio = {ratio:.2f}"
        )
        checks = []
        if s["long_enough"]:
            checks.append("行程足够")
        else:
            checks.append(f"[!] 行程过短 ({travel*100:.1f} < {MIN_TRAVEL_M*100:.0f} cm)")
        if s["clean"]:
            checks.append("单轴清晰")
        else:
            frac = abs(dom) / travel if travel else 0.0
            checks.append(f"[!] 动作不够单轴 ({frac:.0%} < {DOMINANT:.0%})")
        if s["correct_axis"]:
            checks.append("轴正确")
        else:
            checks.append("[!] 轴错了")
        if s["correct_sign"]:
            checks.append("方向正确")
        else:
            checks.append("[!] 方向反了")
        if s["low_residual"]:
            checks.append("残差低")
        else:
            checks.append(f"[!] 残差过大 ({ratio:.0%} > {RESIDUAL:.0%})")
        verdict = "✓" if s["ok"] else "✗"
        print(f"  {verdict} {' / '.join(checks)}")
        if not s["ok"]:
            all_ok = False
    return all_ok


def _axis_map_from_R(R) -> dict:
    """Extract YAML axis_map from a rotation matrix."""
    mapping = {}
    for robot_axis in range(3):
        row = R[robot_axis, :]
        pico_axis = int(np.argmax(np.abs(row)))
        sign = float(np.sign(row[pico_axis]))
        mapping[AXIS_NAME[robot_axis]] = [AXIS_NAME[pico_axis], sign]
    return mapping


def _write_config(config_path, axis_map) -> None:
    """Rewrite only the three axis_map lines, in place.

    A yaml.safe_load/yaml.dump round-trip would drop every comment in
    teleop.yaml — including the calibration notes that explain why each limit
    and offset has the value it does. So edit the axis_map block textually and
    leave the rest of the file byte-identical.
    """
    lines = Path(config_path).read_text(encoding="utf-8").splitlines(keepends=True)
    out: list[str] = []
    i = 0
    replaced = False
    while i < len(lines):
        line = lines[i]
        out.append(line)
        if not replaced and line.strip() == "axis_map:":
            indent = " " * (len(line) - len(line.lstrip()) + 2)
            i += 1
            # Consume the existing x/y/z entries (any order, comments included).
            while i < len(lines):
                stripped = lines[i].strip()
                if stripped.split(":", 1)[0].strip() in {"x", "y", "z"} and ":" in stripped:
                    i += 1
                    continue
                break
            for axis in AXIS_NAME:
                src, sign = axis_map[axis]
                out.append(f'{indent}{axis}: ["{src}", {sign:.1f}]\n')
            replaced = True
            continue
        i += 1
    if not replaced:
        raise RuntimeError(f"{config_path} 里找不到 transform.axis_map — 请手动写入")
    Path(config_path).write_text("".join(out), encoding="utf-8")


def _resolve_tracker_config(vive_cfg: dict) -> Path:
    """Resolve tracker_config with the same base directory as VivePacketSource."""
    tracker_cfg_path = Path(
        vive_cfg.get("tracker_config") or (ROOT / "config" / "tracker.yaml")
    )
    return tracker_cfg_path if tracker_cfg_path.is_absolute() else ROOT / tracker_cfg_path


def main() -> int:
    parser = argparse.ArgumentParser(description="标定 VIVE tracker→robot 坐标映射")
    parser.add_argument(
        "--config",
        default=str(ROOT / "config" / "teleop.yaml"),
        help="teleop.yaml 路径",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="自动写入求解的 axis_map 到 config（覆盖旧的）",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.is_file():
        print(f"错误: {config_path} 不存在")
        return 1
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}

    vive_cfg = dict(config.get("vive") or {})
    tracker_cfg_path = _resolve_tracker_config(vive_cfg)
    if not tracker_cfg_path.is_file():
        print(f"错误: tracker config {tracker_cfg_path} 不存在")
        return 1

    manager = TrackerManager.from_config(tracker_cfg_path)
    manager.start()

    try:
        recorded = _measure(manager, config)
        R, stats = _solve(recorded)
        all_ok = _report_solution(R, stats)
    finally:
        manager.stop()

    print("\n" + "=" * 72)
    axis_map = _axis_map_from_R(R)
    print("axis_map (YAML):")
    print(yaml.dump({"axis_map": axis_map}, allow_unicode=True, default_flow_style=False, sort_keys=False))

    if not all_ok:
        print("[!] 验证未全部通过 — 检查操作是否准确（单轴移动、停稳、head 不动）")
        print("    如果结果仍不对，检查 vr_space、head_tracker_to_body 和 tracker 安装方向")
        return 1

    print("✓ 验证通过 — 此 axis_map 应当产生正确的映射")
    if args.write:
        _write_config(config_path, axis_map)
        print(f"\n已写入 {config_path}")
        print("重启 teleop app 应用新配置")
    else:
        print("\n未写入（传 --write 自动写入）")
        print(f"手动写入: 编辑 {config_path} 的 transform.axis_map 字段")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
