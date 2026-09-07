"""libsurvive backend for VIVE Tracker 3.0.

Priority:
1. pysurvive Python binding
2. ctypes against libsurvive.so (simple API)
3. Do not add a C++ bridge here

Importing this module must not require libsurvive.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import logging
import os
import queue
import sys
import threading
from pathlib import Path
from typing import Any, Optional

from vive_tracking.backend import TrackerBackend
from vive_tracking.exceptions import BackendInitError, BackendNotAvailableError, ConfigError
from vive_tracking.pose_utils import enforce_quaternion_continuity, normalize_quaternion
from vive_tracking.timestamp import host_monotonic_ns
from vive_tracking.types import Quaternion, TrackerDevice, TrackerPose, Vec3

LOGGER = logging.getLogger(__name__)

_INSTALL_HINT = """Libsurvive backend requested but libsurvive
is not available.

Please run:

./scripts/install_libsurvive.sh

Then rerun the program.
"""

_RUNTIME_ALIASES = {"T0", "T1", "T2", "T3", "WM0", "WM1", "WM2", "WM3", "HMD"}
_POLL_S = 0.002
_STALE_S = 0.25
_VIVE_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
_PATH_FLAGS = {"--steamvr-calibration", "--init-configfile", "--configfile"}
_UNUSED_LH_DISABLE_FROM = 2
_UNUSED_LH_DISABLE_TO = 16
_SURVIVE_OBJECT_LIGHTHOUSE = 1


class LibsurviveBackend(TrackerBackend):
    def __init__(self, config: dict[str, Any], trackers: list[dict[str, Any]]) -> None:
        self._config = config
        self._ls_cfg = dict(config.get("libsurvive") or {})
        queue_cfg = dict(config.get("queue") or {})
        self._trackers = [t for t in trackers if t.get("enabled", True)]
        if not self._trackers:
            raise ConfigError("No enabled trackers in config")
        max_size = int(queue_cfg.get("max_size", 1000))
        if max_size < 1:
            raise ConfigError("queue.max_size must be >= 1")
        self._queue: queue.Queue[TrackerPose] = queue.Queue(maxsize=max_size)
        self._latest: dict[str, TrackerPose] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._initialized = False
        self._drop_count = 0
        self._seq: dict[str, int] = {}
        self._prev_quat: dict[str, Quaternion] = {}
        self._impl: Optional[_SurviveImpl] = None
        self._logical_by_serial = {
            str(t["serial"]): str(t.get("logical_name", t["serial"])) for t in self._trackers
        }

    def initialize(self) -> None:
        impl = _open_survive(self._ls_cfg)
        try:
            impl.start()
        except Exception as exc:
            impl.close()
            raise BackendInitError(f"libsurvive failed to start: {exc}") from exc
        self._impl = impl
        self._initialized = True
        LOGGER.info("LibsurviveBackend initialized via %s", impl.kind)

    def start(self) -> None:
        if not self._initialized:
            self.initialize()
        if self._running:
            return
        self._stop.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._run,
            name="libsurvive-tracker-reader",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._impl is not None:
            self._impl.close()
            self._impl = None
        LOGGER.info("LibsurviveBackend stopped queue_drop_count=%d", self._drop_count)

    def is_running(self) -> bool:
        return self._running

    def list_devices(self) -> list[TrackerDevice]:
        discovered: dict[str, TrackerDevice] = {}
        if self._impl is not None:
            for serial, valid in self._impl.list_serials():
                discovered[serial] = TrackerDevice(
                    serial=serial,
                    logical_name=self._logical_by_serial.get(serial, serial),
                    tracking_valid=valid,
                    source="libsurvive",
                )
        with self._lock:
            for serial, pose in self._latest.items():
                discovered[serial] = TrackerDevice(
                    serial=serial,
                    logical_name=pose.logical_name,
                    tracking_valid=pose.tracking_valid,
                    source="libsurvive",
                )
        return list(discovered.values())

    def get_latest_pose(self, serial: Optional[str] = None) -> Optional[TrackerPose]:
        with self._lock:
            if serial is None:
                if not self._latest:
                    return None
                return next(iter(self._latest.values()))
            return self._latest.get(serial)

    def wait_for_pose(self, timeout: Optional[float] = None) -> Optional[TrackerPose]:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def queue_drop_count(self) -> int:
        return self._drop_count

    def _run(self) -> None:
        assert self._impl is not None
        while not self._stop.is_set():
            sample = self._impl.poll(_POLL_S)
            if sample is None:
                continue
            pose = self._to_pose(sample)
            self._publish(pose)

    def _to_pose(self, sample: _PoseSample) -> TrackerPose:
        serial = sample.serial
        if serial in _RUNTIME_ALIASES:
            LOGGER.warning(
                "libsurvive reported runtime alias %s; prefer LHR-XXXXXXXX as the permanent ID",
                serial,
            )
        logical_name = self._logical_by_serial.get(serial, serial)
        orientation = enforce_quaternion_continuity(
            normalize_quaternion(sample.orientation),
            self._prev_quat.get(serial),
        )
        self._prev_quat[serial] = orientation
        self._seq[serial] = self._seq.get(serial, 0) + 1
        return TrackerPose(
            serial=serial,
            logical_name=logical_name,
            position_m=sample.position,
            orientation=orientation,
            sensor_timestamp=sample.sensor_timestamp,
            host_monotonic_ns=host_monotonic_ns(),
            tracking_valid=sample.tracking_valid,
            sequence_id=self._seq[serial],
            source="libsurvive",
        )

    def _publish(self, pose: TrackerPose) -> None:
        with self._lock:
            self._latest[pose.serial] = pose
        try:
            self._queue.put_nowait(pose)
        except queue.Full:
            self._drop_count += 1
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(pose)
            except queue.Full:
                LOGGER.warning("libsurvive pose queue still full serial=%s", pose.serial)


class _PoseSample:
    __slots__ = ("serial", "position", "orientation", "sensor_timestamp", "tracking_valid")

    def __init__(
        self,
        serial: str,
        position: Vec3,
        orientation: Quaternion,
        sensor_timestamp: Optional[float],
        tracking_valid: bool,
    ) -> None:
        self.serial = serial
        self.position = position
        self.orientation = orientation
        self.sensor_timestamp = sensor_timestamp
        self.tracking_valid = tracking_valid


class _SurviveImpl:
    kind: str

    def start(self) -> None:
        raise NotImplementedError

    def poll(self, timeout_s: float) -> Optional[_PoseSample]:
        raise NotImplementedError

    def list_serials(self) -> list[tuple[str, bool]]:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


def resolve_libsurvive_extra_args(
    raw: list[Any],
    *,
    config_dir: Optional[Path] = None,
) -> list[str]:
    """Resolve relative libsurvive paths; disable unused LH channels for gen 1/2."""
    base = Path(config_dir) if config_dir is not None else _VIVE_CONFIG_DIR
    extra = [str(a) for a in raw]
    out: list[str] = []
    pending_path = False
    for item in extra:
        if pending_path:
            path = Path(item)
            if not path.is_absolute():
                item = str((base / path).resolve())
            pending_path = False
        elif item in _PATH_FLAGS:
            pending_path = True
        out.append(item)
    gen = _lighthouse_gen_from_args(out)
    # Gen 1 only uses LH 0/1. Neighbor-room 2.0 stations occupy modes 2–16
    # (this table: LHB-9270D2A9 / LHB-8D9A5499). Disable those channels so
    # --lighthouse-gen 1 cannot still lock onto them as LH 0/1.
    if gen in {1, 2}:
        present = set(out)
        for idx in range(_UNUSED_LH_DISABLE_FROM, _UNUSED_LH_DISABLE_TO):
            flag = f"--lighthouse-{idx}-disable"
            if flag not in present:
                out.append(flag)
    return out


def _lighthouse_gen_from_args(args: list[str]) -> Optional[int]:
    for i, item in enumerate(args):
        if item == "--lighthouse-gen" and i + 1 < len(args):
            try:
                return int(args[i + 1])
            except ValueError:
                return None
    return None


def _open_survive(ls_cfg: dict[str, Any]) -> _SurviveImpl:
    pysurvive = _try_import_pysurvive()
    if pysurvive is not None:
        return _PysurviveImpl(pysurvive, ls_cfg)
    so_path = _find_libsurvive_so()
    if so_path is not None:
        LOGGER.warning("pysurvive unavailable; using ctypes against %s", so_path)
        return _CtypesImpl(so_path, ls_cfg)
    raise BackendNotAvailableError(_INSTALL_HINT)


def _try_import_pysurvive() -> Any:
    try:
        import pysurvive  # type: ignore
    except Exception as exc:
        LOGGER.info("pysurvive not importable: %s", exc)
        return None
    return pysurvive


def _find_libsurvive_so() -> Optional[Path]:
    for name in ("survive", "libsurvive"):
        found = ctypes.util.find_library(name)
        if found:
            return Path(found)
    extra = [
        Path("/usr/local/lib/libsurvive.so"),
        Path("/usr/lib/libsurvive.so"),
        Path("/usr/lib/x86_64-linux-gnu/libsurvive.so"),
        Path.home() / "third_party/libsurvive/libsurvive.so",
        Path.home() / "third_party/libsurvive/bin/libsurvive.so",
        Path.home() / "third_party/libsurvive/build/libsurvive.so",
    ]
    env = os.environ.get("LIBSURVIVE_SO")
    if env:
        extra.insert(0, Path(env))
    for path in extra:
        if path.is_file():
            return path
    return None


def _extract_serial(obj: Any, fallback_name: str) -> str:
    for attr in ("serial", "serial_number", "Serial", "codename"):
        value = getattr(obj, attr, None)
        if callable(value):
            try:
                value = value()
            except Exception:
                value = None
        if isinstance(value, bytes):
            value = value.decode("utf-8", errors="replace")
        if isinstance(value, str) and value.strip():
            text = value.strip()
            if text.upper().startswith("LHR-") or text not in _RUNTIME_ALIASES:
                return text
    return fallback_name


def _extract_sensor_timestamp(obj: Any, pose_result: Any) -> Optional[float]:
    if isinstance(pose_result, tuple) and len(pose_result) >= 2:
        stamp = pose_result[1]
        try:
            return float(stamp)
        except (TypeError, ValueError):
            pass
    for attr in ("PoseTime", "Time", "timecode", "last_update_time"):
        value = getattr(obj, attr, None)
        if value is None:
            continue
        try:
            if callable(value):
                value = value()
            return float(value)
        except Exception:
            continue
    return None


class _PysurviveImpl(_SurviveImpl):
    kind = "pysurvive"

    def __init__(self, pysurvive: Any, ls_cfg: dict[str, Any]) -> None:
        self._pysurvive = pysurvive
        self._ls_cfg = ls_cfg
        self._ctx: Any = None

    def start(self) -> None:
        extra = resolve_libsurvive_extra_args(self._ls_cfg.get("extra_args") or [])
        LOGGER.info("libsurvive argv extra=%s", extra)
        argv = [sys.argv[0], *extra]
        context_cls = getattr(self._pysurvive, "SimpleContext", None)
        if context_cls is None:
            raise BackendInitError("pysurvive.SimpleContext is missing")
        self._ctx = context_cls(argv)

    def poll(self, timeout_s: float) -> Optional[_PoseSample]:
        ctx = self._ctx
        if ctx is None:
            return None
        running = getattr(ctx, "Running", None)
        if callable(running) and not running():
            threading.Event().wait(timeout_s)
            return None
        updated = ctx.NextUpdated()
        if updated is None:
            threading.Event().wait(timeout_s)
            return None
        name = str(updated.Name()) if callable(getattr(updated, "Name", None)) else str(updated)
        serial = _extract_serial(updated, name)
        pose_result = updated.Pose()
        pose_obj = pose_result[0] if isinstance(pose_result, tuple) else pose_result
        pos = _as_xyz(getattr(pose_obj, "Pos", pose_obj))
        rot = _as_wxyz(getattr(pose_obj, "Rot", None))
        sensor_ts = _extract_sensor_timestamp(updated, pose_result)
        return _PoseSample(
            serial=serial,
            position=Vec3(*pos),
            orientation=Quaternion(*rot),
            sensor_timestamp=sensor_ts,
            tracking_valid=True,
        )

    def list_serials(self) -> list[tuple[str, bool]]:
        ctx = self._ctx
        if ctx is None or not hasattr(ctx, "Objects"):
            return []
        out: list[tuple[str, bool]] = []
        for obj in ctx.Objects():
            name = str(obj.Name()) if callable(getattr(obj, "Name", None)) else str(obj)
            out.append((_extract_serial(obj, name), True))
        return out

    def close(self) -> None:
        ctx = self._ctx
        self._ctx = None
        if ctx is None:
            return
        for name in ("Close", "close", "Destroy"):
            fn = getattr(ctx, name, None)
            if callable(fn):
                try:
                    fn()
                except Exception as exc:
                    LOGGER.warning("pysurvive close via %s failed: %s", name, exc)
                return


class SurvivePoseC(ctypes.Structure):
    _fields_ = [
        ("Pos", ctypes.c_double * 3),
        ("Rot", ctypes.c_double * 4),
    ]


class _CtypesImpl(_SurviveImpl):
    kind = "ctypes"

    def __init__(self, so_path: Path, ls_cfg: dict[str, Any]) -> None:
        self._so_path = so_path
        self._ls_cfg = ls_cfg
        self._lib: Optional[ctypes.CDLL] = None
        self._ctx = None

    def start(self) -> None:
        lib = ctypes.CDLL(str(self._so_path))
        lib.survive_simple_init.restype = ctypes.c_void_p
        lib.survive_simple_init.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
        lib.survive_simple_start_thread.argtypes = [ctypes.c_void_p]
        lib.survive_simple_is_running.restype = ctypes.c_bool
        lib.survive_simple_is_running.argtypes = [ctypes.c_void_p]
        lib.survive_simple_get_next_updated.restype = ctypes.c_void_p
        lib.survive_simple_get_next_updated.argtypes = [ctypes.c_void_p]
        lib.survive_simple_object_name.restype = ctypes.c_char_p
        lib.survive_simple_object_name.argtypes = [ctypes.c_void_p]
        lib.survive_simple_serial_number.restype = ctypes.c_char_p
        lib.survive_simple_serial_number.argtypes = [ctypes.c_void_p]
        lib.survive_simple_object_get_latest_pose.restype = ctypes.c_double
        lib.survive_simple_object_get_latest_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(SurvivePoseC),
        ]
        lib.survive_simple_close.argtypes = [ctypes.c_void_p]
        lib.survive_simple_get_first_object.restype = ctypes.c_void_p
        lib.survive_simple_get_first_object.argtypes = [ctypes.c_void_p]
        lib.survive_simple_get_next_object.restype = ctypes.c_void_p
        lib.survive_simple_get_next_object.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.survive_simple_object_get_type.restype = ctypes.c_int
        lib.survive_simple_object_get_type.argtypes = [ctypes.c_void_p]
        extra = resolve_libsurvive_extra_args(self._ls_cfg.get("extra_args") or [])
        LOGGER.info("libsurvive argv extra=%s", extra)
        argv_list = [sys.argv[0].encode(), *[a.encode() for a in extra]]
        argv = (ctypes.c_char_p * len(argv_list))(*argv_list)
        ctx = lib.survive_simple_init(len(argv_list), argv)
        if not ctx:
            raise BackendInitError("survive_simple_init returned NULL")
        lib.survive_simple_start_thread(ctx)
        self._lib = lib
        self._ctx = ctx

    def poll(self, timeout_s: float) -> Optional[_PoseSample]:
        lib = self._lib
        ctx = self._ctx
        if lib is None or ctx is None:
            return None
        obj = lib.survive_simple_get_next_updated(ctx)
        if not obj:
            threading.Event().wait(timeout_s)
            return None
        if lib.survive_simple_object_get_type(obj) == _SURVIVE_OBJECT_LIGHTHOUSE:
            return None
        raw_serial = lib.survive_simple_serial_number(obj)
        raw_name = lib.survive_simple_object_name(obj)
        serial_text = (
            raw_serial.decode("utf-8", errors="replace") if raw_serial else ""
        ).strip()
        name_text = (
            raw_name.decode("utf-8", errors="replace") if raw_name else ""
        ).strip()
        serial = serial_text or name_text or "unknown"
        pose = SurvivePoseC()
        timecode = lib.survive_simple_object_get_latest_pose(obj, ctypes.byref(pose))
        sensor_ts: Optional[float]
        try:
            sensor_ts = float(timecode) if timecode else None
        except (TypeError, ValueError):
            sensor_ts = None
        return _PoseSample(
            serial=serial,
            position=Vec3(float(pose.Pos[0]), float(pose.Pos[1]), float(pose.Pos[2])),
            orientation=Quaternion(
                float(pose.Rot[0]),
                float(pose.Rot[1]),
                float(pose.Rot[2]),
                float(pose.Rot[3]),
            ),
            sensor_timestamp=sensor_ts,
            tracking_valid=True,
        )

    def list_serials(self) -> list[tuple[str, bool]]:
        lib = self._lib
        ctx = self._ctx
        if lib is None or ctx is None:
            return []
        out: list[tuple[str, bool]] = []
        obj = lib.survive_simple_get_first_object(ctx)
        while obj:
            if lib.survive_simple_object_get_type(obj) == _SURVIVE_OBJECT_LIGHTHOUSE:
                obj = lib.survive_simple_get_next_object(ctx, obj)
                continue
            raw_serial = lib.survive_simple_serial_number(obj)
            raw_name = lib.survive_simple_object_name(obj)
            serial_text = (
                raw_serial.decode("utf-8", errors="replace") if raw_serial else ""
            ).strip()
            name_text = (
                raw_name.decode("utf-8", errors="replace") if raw_name else ""
            ).strip()
            serial = serial_text or name_text or "unknown"
            pose = SurvivePoseC()
            timecode = lib.survive_simple_object_get_latest_pose(obj, ctypes.byref(pose))
            out.append((serial, bool(timecode)))
            obj = lib.survive_simple_get_next_object(ctx, obj)
        return out

    def close(self) -> None:
        lib = self._lib
        ctx = self._ctx
        self._lib = None
        self._ctx = None
        if lib is not None and ctx:
            try:
                lib.survive_simple_close(ctx)
            except Exception as exc:
                LOGGER.warning("survive_simple_close failed: %s", exc)


def _as_xyz(value: Any) -> tuple[float, float, float]:
    if value is None:
        raise BackendInitError("libsurvive pose is missing Pos")
    if hasattr(value, "__getitem__"):
        return (float(value[0]), float(value[1]), float(value[2]))
    raise BackendInitError(f"Unsupported Pos type: {type(value)!r}")


def _as_wxyz(value: Any) -> tuple[float, float, float, float]:
    if value is None:
        raise BackendInitError("libsurvive pose is missing Rot (expected wxyz)")
    if hasattr(value, "__getitem__"):
        return (float(value[0]), float(value[1]), float(value[2]), float(value[3]))
    raise BackendInitError(f"Unsupported Rot type: {type(value)!r}")
