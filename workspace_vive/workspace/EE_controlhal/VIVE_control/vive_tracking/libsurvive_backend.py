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
_CLOSE_TIMEOUT_S = 1.0
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
        if _is_runtime_alias(serial):
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
    lighthouse_calibration: bool = False,
) -> list[str]:
    """Resolve relative libsurvive paths; disable unused LH channels for gen 1/2."""
    base = Path(config_dir) if config_dir is not None else _VIVE_CONFIG_DIR
    extra = [str(a) for a in raw]
    if lighthouse_calibration:
        # Calibration must be able to update lighthouse poses.  The normal
        # teleop configuration deliberately contains this flag to keep the
        # saved geometry fixed.
        extra = [item for item in extra if item != "--disable-calibrate"]
        if "--force-calibrate" not in extra:
            extra.append("--force-calibrate")
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


def _decode_text(value: Any) -> str:
    """Decode libsurvive's ``char *`` values without producing ``b'...'``."""
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").strip()
    if isinstance(value, (bytearray, memoryview)):
        return bytes(value).decode("utf-8", errors="replace").strip()
    if isinstance(value, str):
        return value.strip()
    return str(value).strip()


def _object_name(obj: Any) -> str:
    name = getattr(obj, "Name", None)
    if callable(name):
        try:
            return _decode_text(name())
        except Exception:
            return ""
    return _decode_text(obj)


def _extract_serial(
    obj: Any,
    fallback_name: str,
    native_serial: Any = None,
) -> str:
    values = [native_serial]
    for attr in ("serial", "serial_number", "Serial", "codename"):
        value = getattr(obj, attr, None)
        if callable(value):
            try:
                value = value()
            except Exception:
                value = None
        values.append(value)
    for value in values:
        text = _decode_text(value)
        if text and (text.upper().startswith("LHR-") or not _is_runtime_alias(text)):
            return text
    return _decode_text(fallback_name)


def _is_runtime_alias(value: str) -> bool:
    """Whether libsurvive has not resolved this object to an LHR serial yet."""
    upper = value.strip().upper()
    return (
        upper in _RUNTIME_ALIASES
        or (upper.startswith("T") and upper[1:].isdigit())
        or (upper.startswith("WM") and upper[2:].isdigit())
    )


def _pysurvive_native_serial(pysurvive: Any, obj: Any) -> Any:
    """Return the simple API serial, which ``SimpleObject`` does not expose."""
    ptr = getattr(obj, "ptr", None)
    if ptr is None:
        return None
    for name in ("simple_serial_number", "survive_simple_serial_number"):
        fn = getattr(pysurvive, name, None)
        if callable(fn):
            try:
                return fn(ptr)
            except Exception:
                continue
    return None


def _pysurvive_object_type(pysurvive: Any, obj: Any) -> Optional[int]:
    """Read an object's simple-API type across pysurvive binding variants."""
    for attr in ("Type", "type", "object_type"):
        value = getattr(obj, attr, None)
        if callable(value):
            try:
                value = value()
            except Exception:
                value = None
        if value is not None:
            try:
                return int(value)
            except (TypeError, ValueError):
                pass

    ptr = getattr(obj, "ptr", None)
    if ptr is None:
        return None
    for name in ("simple_object_get_type", "survive_simple_object_get_type"):
        fn = getattr(pysurvive, name, None)
        if callable(fn):
            try:
                return int(fn(ptr))
            except (TypeError, ValueError):
                return None
            except Exception:
                continue
    return None


def _is_lighthouse(pysurvive: Any, obj: Any, name: str) -> bool:
    object_type = _pysurvive_object_type(pysurvive, obj)
    lighthouse_type = getattr(
        pysurvive,
        "SimpleObject_LIGHTHOUSE",
        getattr(pysurvive, "SurviveSimpleObject_LIGHTHOUSE", _SURVIVE_OBJECT_LIGHTHOUSE),
    )
    try:
        if object_type == int(lighthouse_type):
            return True
    except (TypeError, ValueError):
        if object_type == _SURVIVE_OBJECT_LIGHTHOUSE:
            return True

    upper_name = _decode_text(name).upper()
    # Lighthouses are named LH0/LH1 or LHB-...; do not confuse a tracker
    # serial such as LHR-XXXXXXXX with a lighthouse.
    return (
        upper_name.startswith("LH")
        and not upper_name.startswith("LHR-")
    )


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


def _has_initialized_pose(pose_result: Any) -> bool:
    """Reject libsurvive's zero/identity placeholder before a poser solves."""
    pose_obj = pose_result[0] if isinstance(pose_result, tuple) else pose_result
    try:
        position = _as_xyz(getattr(pose_obj, "Pos", pose_obj))
        rotation = _as_wxyz(getattr(pose_obj, "Rot", None))
    except (TypeError, ValueError, BackendInitError):
        return False
    epsilon = 1e-8
    position_is_zero = all(abs(value) <= epsilon for value in position)
    rotation_is_identity = (
        abs(abs(rotation[0]) - 1.0) <= epsilon
        and all(abs(value) <= epsilon for value in rotation[1:])
    )
    rotation_is_zero = all(abs(value) <= epsilon for value in rotation)
    return not (position_is_zero and (rotation_is_identity or rotation_is_zero))


def _close_with_timeout(close_fn: Any, name: str) -> None:
    """Do not let a third-party libsurvive shutdown block teleop termination."""
    completed = threading.Event()

    def _run() -> None:
        try:
            close_fn()
        except Exception as exc:
            LOGGER.warning("pysurvive close via %s failed: %s", name, exc)
        finally:
            completed.set()

    thread = threading.Thread(target=_run, name="pysurvive-close", daemon=True)
    thread.start()
    if not completed.wait(_CLOSE_TIMEOUT_S):
        LOGGER.warning("pysurvive close via %s timed out after %.1f s", name, _CLOSE_TIMEOUT_S)


class _PysurviveImpl(_SurviveImpl):
    kind = "pysurvive"

    def __init__(self, pysurvive: Any, ls_cfg: dict[str, Any]) -> None:
        self._pysurvive = pysurvive
        self._ls_cfg = ls_cfg
        self._ctx: Any = None
        self._known_serials: dict[str, bool] = {}
        self._last_sensor_timestamp: dict[str, float] = {}
        self._last_polled_serial: Optional[str] = None

    def start(self) -> None:
        extra = resolve_libsurvive_extra_args(
            self._ls_cfg.get("extra_args") or [],
            lighthouse_calibration=bool(self._ls_cfg.get("lighthouse_calibration", False)),
        )
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

        # pysurvive's ``SimpleContext.NextUpdated()`` has blocked indefinitely
        # on some installed bindings when no tracker pose arrives.  The simple
        # API exposes the latest pose for every live object, so poll that API
        # directly and use the pose timecode as the update marker.  This keeps
        # the reader thread bounded and lets ``LibsurviveBackend.stop()`` join.
        objects = self._tracker_objects(ctx)
        if objects:
            start = 0
            if self._last_polled_serial is not None:
                for index, (serial, _obj) in enumerate(objects):
                    if serial == self._last_polled_serial:
                        start = (index + 1) % len(objects)
                        break

            for offset in range(len(objects)):
                serial, obj = objects[(start + offset) % len(objects)]
                sample = self._sample_from_object(serial, obj)
                if sample is not None:
                    self._last_polled_serial = serial
                    return sample

        threading.Event().wait(timeout_s)
        return None

    def list_serials(self) -> list[tuple[str, bool]]:
        ctx = self._ctx
        if ctx is None:
            return []
        # ``SimpleContext.Objects()`` is a startup-time snapshot in pysurvive
        # 1.1.204. USB trackers can appear after that snapshot, so enumerate
        # the current C simple-API list on every request.
        discovered = dict(self._known_serials)
        for serial, obj in self._tracker_objects(ctx):
            discovered[serial] = self._object_has_valid_pose(obj)
        self._known_serials = discovered
        return sorted(discovered.items())

    def _tracker_objects(self, ctx: Any) -> list[tuple[str, Any]]:
        """Return live tracker objects keyed by their stable serial number."""
        objects: list[tuple[str, Any]] = []
        seen_serials: set[str] = set()
        for obj in self._current_objects(ctx):
            name = _object_name(obj)
            if _is_lighthouse(self._pysurvive, obj, name):
                continue
            serial = _extract_serial(
                obj,
                name,
                _pysurvive_native_serial(self._pysurvive, obj),
            )
            # T20/T21 (and WMn) are libsurvive startup aliases. They may be
            # emitted for a few reader iterations before the driver exposes
            # its permanent LHR serial. Never publish them upstream: a later
            # LHR update would otherwise leave an alias duplicate in _latest.
            if not serial or _is_runtime_alias(serial) or serial in seen_serials:
                continue
            seen_serials.add(serial)
            objects.append((serial, obj))
        return objects

    def _sample_from_object(self, serial: str, obj: Any) -> Optional[_PoseSample]:
        pose_result = self._object_pose(obj)
        if pose_result is None:
            self._known_serials[serial] = False
            return None
        pose_obj = pose_result[0] if isinstance(pose_result, tuple) else pose_result
        sensor_ts = _extract_sensor_timestamp(obj, pose_result)
        if sensor_ts is None or sensor_ts <= 0.0:
            self._known_serials[serial] = False
            return None
        if not _has_initialized_pose(pose_result):
            self._known_serials[serial] = False
            return None

        self._known_serials[serial] = True
        if self._last_sensor_timestamp.get(serial) == sensor_ts:
            return None
        try:
            pos = _as_xyz(getattr(pose_obj, "Pos", pose_obj))
            rot = _as_wxyz(getattr(pose_obj, "Rot", None))
        except (TypeError, ValueError, BackendInitError) as exc:
            LOGGER.debug("pysurvive pose decode failed serial=%s: %s", serial, exc)
            return None
        self._last_sensor_timestamp[serial] = sensor_ts
        return _PoseSample(
            serial=serial,
            position=Vec3(*pos),
            orientation=Quaternion(*rot),
            sensor_timestamp=sensor_ts,
            tracking_valid=True,
        )

    @staticmethod
    def _object_pose(obj: Any) -> Any:
        pose = getattr(obj, "Pose", None)
        if not callable(pose):
            return None
        try:
            return pose()
        except Exception as exc:
            LOGGER.debug("pysurvive pose read failed: %s", exc)
            return None

    def _object_has_valid_pose(self, obj: Any) -> bool:
        pose_result = self._object_pose(obj)
        if pose_result is None:
            return False
        sensor_ts = _extract_sensor_timestamp(obj, pose_result)
        return (
            sensor_ts is not None
            and sensor_ts > 0.0
            and _has_initialized_pose(pose_result)
        )

    def _current_objects(self, ctx: Any) -> list[Any]:
        """Return live simple-API objects, falling back to the binding snapshot."""
        objects: list[Any] = []
        ptr = getattr(ctx, "ptr", None)
        first = getattr(self._pysurvive, "simple_get_first_object", None)
        next_object = getattr(self._pysurvive, "simple_get_next_object", None)
        wrapper = getattr(self._pysurvive, "SimpleObject", None)
        if ptr is not None and callable(first) and callable(next_object):
            try:
                current = first(ptr)
                # An upper bound protects against a malformed third-party
                # binding returning the same non-null object indefinitely.
                for _ in range(1024):
                    if not current:
                        break
                    try:
                        obj = wrapper(current) if callable(wrapper) else current
                    except Exception:
                        obj = current
                    objects.append(obj)
                    current = next_object(ptr, current)
                # The raw simple API is authoritative. ``Objects()`` can be
                # an earlier snapshot whose entries retain runtime aliases
                # such as T20/T21, creating duplicate devices beside LHR IDs.
                return objects
            except Exception as exc:
                LOGGER.debug("pysurvive live object enumeration failed: %s", exc)

        snapshot = getattr(ctx, "Objects", None)
        if callable(snapshot):
            try:
                objects.extend(snapshot())
            except Exception as exc:
                LOGGER.debug("pysurvive object snapshot failed: %s", exc)
        return objects

    def close(self) -> None:
        ctx = self._ctx
        self._ctx = None
        if ctx is None:
            return
        for name in ("Close", "close", "Destroy"):
            fn = getattr(ctx, name, None)
            if callable(fn):
                _close_with_timeout(fn, name)
                return
        ptr = getattr(ctx, "ptr", None)
        close = getattr(self._pysurvive, "simple_close", None)
        if ptr is not None and callable(close):
            _close_with_timeout(lambda: close(ptr), "simple_close")


class SurvivePoseFloatC(ctypes.Structure):
    """``SurvivePose`` when libsurvive was built with ``FLT=float``."""
    _fields_ = [
        ("Pos", ctypes.c_float * 3),
        ("Rot", ctypes.c_float * 4),
    ]


class SurvivePoseDoubleC(ctypes.Structure):
    """``SurvivePose`` when libsurvive was built with ``USE_DOUBLE``."""
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
        scalar = str(ls_cfg.get("ctypes_scalar", "double")).strip().lower()
        if scalar == "float":
            self._scalar_ctype = ctypes.c_float
            self._pose_type = SurvivePoseFloatC
        elif scalar == "double":
            self._scalar_ctype = ctypes.c_double
            self._pose_type = SurvivePoseDoubleC
        else:
            raise BackendInitError(
                "libsurvive.ctypes_scalar must be 'float' or 'double'"
            )
        self._scalar_name = scalar

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
        lib.survive_simple_object_get_latest_pose.restype = self._scalar_ctype
        lib.survive_simple_object_get_latest_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(self._pose_type),
        ]
        verify_flt_size = getattr(lib, "survive_verify_FLT_size", None)
        if verify_flt_size is not None:
            verify_flt_size.argtypes = [ctypes.c_uint32]
            verify_flt_size.restype = None
            # libsurvive fails immediately if this does not match the shared
            # object's ABI, which is preferable to decoding corrupt poses.
            verify_flt_size(ctypes.sizeof(self._scalar_ctype))
        lib.survive_simple_close.argtypes = [ctypes.c_void_p]
        lib.survive_simple_get_first_object.restype = ctypes.c_void_p
        lib.survive_simple_get_first_object.argtypes = [ctypes.c_void_p]
        lib.survive_simple_get_next_object.restype = ctypes.c_void_p
        lib.survive_simple_get_next_object.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.survive_simple_object_get_type.restype = ctypes.c_int
        lib.survive_simple_object_get_type.argtypes = [ctypes.c_void_p]
        extra = resolve_libsurvive_extra_args(
            self._ls_cfg.get("extra_args") or [],
            lighthouse_calibration=bool(self._ls_cfg.get("lighthouse_calibration", False)),
        )
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
        pose = self._pose_type()
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
            pose = self._pose_type()
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
