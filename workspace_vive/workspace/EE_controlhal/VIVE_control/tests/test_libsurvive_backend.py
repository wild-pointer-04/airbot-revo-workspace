"""pysurvive binding compatibility tests without VIVE hardware."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

from vive_tracking.libsurvive_backend import LibsurviveBackend, _PysurviveImpl


class _Pose:
    Pos = (1.0, 2.0, 3.0)
    Rot = (1.0, 0.0, 0.0, 0.0)


class _UninitializedPose:
    Pos = (0.0, 0.0, 0.0)
    Rot = (1.0, 0.0, 0.0, 0.0)


class _ZeroQuaternionPose:
    Pos = (0.0, 0.0, 0.0)
    Rot = (0.0, 0.0, 0.0, 0.0)


class _Object:
    def __init__(
        self,
        name: bytes,
        serial: bytes,
        kind: int,
        *,
        pose_time: float = 42.0,
        pose_type: type[_Pose] = _Pose,
    ) -> None:
        self.name = name
        self.serial = serial
        self.kind = kind
        self.pose_time = pose_time
        self.pose_type = pose_type
        self.ptr = self

    def Name(self) -> bytes:
        return self.name

    def Pose(self) -> tuple[_Pose, float]:
        return self.pose_type(), self.pose_time


class _Context:
    def __init__(
        self,
        _argv: list[str],
        objects: list[_Object],
        snapshot_objects: list[_Object] | None = None,
    ) -> None:
        self.ptr = self
        self._objects = objects
        self._snapshot_objects = snapshot_objects or []
        self._updates = list(objects)

    def Running(self) -> bool:
        return True

    def NextUpdated(self) -> _Object | None:
        raise AssertionError("poll() must not use the blocking NextUpdated() API")

    def Objects(self) -> list[_Object]:
        # This models pysurvive 1.1.204: only objects present before its
        # background discovery begins exist in this Python-side snapshot.
        return self._snapshot_objects


def _fake_pysurvive(
    objects: list[_Object],
    snapshot_objects: list[_Object] | None = None,
) -> SimpleNamespace:
    contexts: list[_Context] = []

    def context(argv: list[str]) -> _Context:
        instance = _Context(argv, objects, snapshot_objects)
        contexts.append(instance)
        return instance

    def first(ctx: _Context) -> _Object | None:
        return ctx._objects[0] if ctx._objects else None

    def next_object(ctx: _Context, current: _Object) -> _Object | None:
        index = ctx._objects.index(current) + 1
        return ctx._objects[index] if index < len(ctx._objects) else None

    module = SimpleNamespace(
        SimpleContext=context,
        SimpleObject=lambda ptr: ptr,
        SimpleObject_LIGHTHOUSE=1,
        simple_get_first_object=first,
        simple_get_next_object=next_object,
        simple_object_get_type=lambda obj: obj.kind,
        simple_serial_number=lambda obj: obj.serial,
        simple_close=lambda ctx: setattr(ctx, "closed", True),
        contexts=contexts,
    )
    return module


def test_pysurvive_enumerates_live_trackers_and_filters_lighthouse() -> None:
    lighthouse_by_type = _Object(b"station-zero", b"station-zero", 1)
    lighthouse_by_name = _Object(b"LH0", b"LH0", 3)
    # Name() is a short runtime alias. The native simple-API serial is the
    # stable ID that must be placed in tracker.yaml.
    tracker_20 = _Object(b"WM0", b"LHR-TRACKER-20", 3)
    tracker_21 = _Object(b"WM1", b"LHR-TRACKER-21", 3)
    module = _fake_pysurvive(
        [lighthouse_by_type, lighthouse_by_name, tracker_20, tracker_21]
    )
    impl = _PysurviveImpl(module, {})
    impl.start()

    assert impl.list_serials() == [
        ("LHR-TRACKER-20", True),
        ("LHR-TRACKER-21", True),
    ]
    first = impl.poll(0.0)
    second = impl.poll(0.0)
    assert first is not None
    assert first.serial == "LHR-TRACKER-20"
    assert first.position.as_list() == [1.0, 2.0, 3.0]
    assert second is not None
    assert second.serial == "LHR-TRACKER-21"
    # No pose timecode changed, so the bounded poll returns without publishing
    # another copy of the same sample.
    assert impl.poll(0.0) is None


def test_pysurvive_marks_discovered_tracker_invalid_without_pose() -> None:
    tracker = _Object(b"WM0", b"LHR-TRACKER-20", 3, pose_time=0.0)
    impl = _PysurviveImpl(_fake_pysurvive([tracker]), {})
    impl.start()

    assert impl.list_serials() == [("LHR-TRACKER-20", False)]
    assert impl.poll(0.0) is None


def test_pysurvive_rejects_zero_identity_pose_even_with_a_timecode() -> None:
    tracker = _Object(
        b"WM0",
        b"LHR-TRACKER-20",
        3,
        pose_type=_UninitializedPose,
    )
    impl = _PysurviveImpl(_fake_pysurvive([tracker]), {})
    impl.start()

    assert impl.list_serials() == [("LHR-TRACKER-20", False)]
    assert impl.poll(0.0) is None


def test_pysurvive_rejects_zero_quaternion_pose_even_with_a_timecode() -> None:
    tracker = _Object(
        b"WM0",
        b"LHR-TRACKER-20",
        3,
        pose_type=_ZeroQuaternionPose,
    )
    impl = _PysurviveImpl(_fake_pysurvive([tracker]), {})
    impl.start()

    assert impl.list_serials() == [("LHR-TRACKER-20", False)]
    assert impl.poll(0.0) is None


def test_pysurvive_uses_live_stable_serial_instead_of_alias_snapshot() -> None:
    live_tracker = _Object(b"WM0", b"LHR-TRACKER-20", 3)
    stale_snapshot = _Object(b"WM0", b"WM0", 3)
    impl = _PysurviveImpl(
        _fake_pysurvive([live_tracker], [stale_snapshot]),
        {},
    )
    impl.start()

    assert impl.list_serials() == [("LHR-TRACKER-20", True)]
    sample = impl.poll(0.0)
    assert sample is not None
    assert sample.serial == "LHR-TRACKER-20"


def test_pysurvive_waits_for_stable_lhr_serial_before_publishing() -> None:
    alias_only_tracker = _Object(b"T20", b"T20", 3)
    impl = _PysurviveImpl(_fake_pysurvive([alias_only_tracker]), {})
    impl.start()

    assert impl.list_serials() == []
    assert impl.poll(0.0) is None


def test_backend_stop_joins_reader_when_pysurvive_has_no_pose_updates() -> None:
    tracker = _Object(b"WM0", b"LHR-TRACKER-20", 3, pose_time=0.0)
    module = _fake_pysurvive([tracker])
    impl = _PysurviveImpl(module, {})
    impl.start()
    backend = LibsurviveBackend(
        {"queue": {"max_size": 1}},
        [{"logical_name": "hand_controller", "serial": "LHR-TRACKER-20"}],
    )
    backend._impl = impl
    backend._initialized = True
    backend.start()
    thread = backend._thread

    backend.stop()

    assert thread is not None
    assert not thread.is_alive()
    assert module.contexts[0].closed is True


def test_pysurvive_close_uses_raw_simple_api() -> None:
    module = _fake_pysurvive([])
    impl = _PysurviveImpl(module, {})
    impl.start()

    impl.close()

    assert module.contexts[0].closed is True


def test_pysurvive_close_does_not_block_on_a_stuck_binding() -> None:
    release = threading.Event()
    module = _fake_pysurvive([])

    def blocking_close(_ctx: _Context) -> None:
        release.wait(5.0)

    module.simple_close = blocking_close
    impl = _PysurviveImpl(module, {})
    impl.start()

    start = time.monotonic()
    impl.close()
    elapsed = time.monotonic() - start
    release.set()

    assert elapsed < 1.5
