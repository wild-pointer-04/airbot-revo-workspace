"""TrackerManager mapping, cache, and last-valid pose."""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from vive_tracking.exceptions import ConfigError, DeviceNotFoundError
from vive_tracking.tracker_manager import TrackerManager, create_backend, load_config

ROOT = Path(__file__).resolve().parents[1]


def _manager(mode: str = "static") -> TrackerManager:
    cfg = {
        "backend": "mock",
        "mock": {
            "mode": mode,
            "update_rate_hz": 80,
            "dropout": {"enabled": mode == "dropout", "duration_ms": 80, "period_s": 0.35},
        },
        "queue": {"max_size": 200},
        "trackers": [
            {"logical_name": "dexterous_hand", "serial": "MOCK-HAND-001", "enabled": True}
        ],
    }
    return TrackerManager(create_backend(cfg), cfg)


def test_from_config_yaml() -> None:
    manager = TrackerManager.from_config(ROOT / "config" / "tracker.mock.yaml")
    manager.start()
    try:
        pose = None
        for _ in range(20):
            pose = manager.get_pose("hand_controller")
            if pose is not None:
                break
            time.sleep(0.02)
        assert pose is not None
        assert pose.serial == "MOCK-HAND-001"
        assert pose.logical_name == "hand_controller"
        devices = manager.list_devices()
        serials = {d.serial for d in devices}
        assert "MOCK-HAND-001" in serials
        assert "MOCK-HEAD-001" in serials
        assert manager.get_pose("body_hmd") is not None
    finally:
        manager.stop()


def test_unknown_logical_name() -> None:
    manager = _manager()
    manager.start()
    try:
        with pytest.raises(DeviceNotFoundError):
            manager.get_pose("does_not_exist")
    finally:
        manager.stop()


def test_last_valid_not_confused_with_invalid() -> None:
    manager = _manager("dropout")
    manager.start()
    try:
        seen_invalid = False
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            pose = manager.wait_for_pose(timeout=0.2)
            if pose is None:
                continue
            if pose.tracking_valid:
                cached = manager.last_valid_pose("dexterous_hand")
                assert cached is not None
                assert cached.tracking_valid is True
            else:
                seen_invalid = True
                last_invalid = pose
                cached = manager.last_valid_pose("dexterous_hand")
                if cached is not None:
                    assert cached.tracking_valid is True
                    assert cached.sequence_id != last_invalid.sequence_id
                break
        assert seen_invalid
    finally:
        manager.stop()


def test_load_config_missing(tmp_path: Path) -> None:
    with pytest.raises(ConfigError):
        load_config(tmp_path / "nope.yaml")


def test_libsurvive_missing_prints_install_hint() -> None:
    from vive_tracking.exceptions import BackendNotAvailableError

    cfg = {
        "backend": "libsurvive",
        "libsurvive": {},
        "queue": {"max_size": 10},
        "trackers": [
            {"logical_name": "dexterous_hand", "serial": "LHR-XXXXXXXX", "enabled": True}
        ],
    }
    try:
        import pysurvive  # noqa: F401
    except Exception:
        pass
    else:
        pytest.skip("pysurvive is installed on this machine")
    manager = TrackerManager(create_backend(cfg), cfg)
    with pytest.raises(BackendNotAvailableError) as err:
        manager.start()
    assert "install_libsurvive.sh" in str(err.value)


def test_live_yaml_uses_gen1_not_other_room_gen2() -> None:
    from vive_tracking.libsurvive_backend import resolve_libsurvive_extra_args

    cfg = load_config(ROOT / "config" / "tracker.yaml")
    raw = [str(a) for a in cfg["libsurvive"]["extra_args"]]
    assert raw[raw.index("--lighthouse-gen") + 1] == "1"
    assert "--steamvr-calibration" not in raw
    assert "--lighthouse-2-disable" in raw
    assert "--lighthouse-3-disable" in raw
    assert "--ootx-ignore-sync-error" in raw
    assert "--force-ootx" not in raw
    assert "--init-configfile" not in raw
    args = resolve_libsurvive_extra_args(raw, config_dir=ROOT / "config")
    calibration = ROOT / "config" / "libsurvive_calibration.json"
    assert args[args.index("--configfile") + 1] == str(calibration)
    for idx in range(2, 16):
        assert f"--lighthouse-{idx}-disable" in args
    assert "--lighthouse-0-disable" not in args
    assert "--lighthouse-1-disable" not in args
