"""Multi-camera capture: OpenCV/UVC, RealSense by serial, plus a mock backend."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    name: str
    backend: str = "realsense"  # opencv | realsense | mock
    serial_number: str = ""
    width: int = 640
    height: int = 480
    fps: int = 30
    enable_depth: bool = False
    device: str = "/dev/video0"


class RealSenseStream:
    """Single RealSense camera capture thread identified by serial number."""

    def __init__(self, config: CameraConfig):
        self._config = config
        self._pipeline = None
        self._frame: Optional[np.ndarray] = None
        self._depth_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self) -> bool:
        try:
            import pyrealsense2 as rs
        except ImportError:
            log.error("pyrealsense2 not installed")
            return False

        try:
            self._pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(self._config.serial_number)
            cfg.enable_stream(
                rs.stream.color,
                self._config.width,
                self._config.height,
                rs.format.rgb8,
                self._config.fps,
            )
            if self._config.enable_depth:
                cfg.enable_stream(
                    rs.stream.depth,
                    self._config.width,
                    self._config.height,
                    rs.format.z16,
                    self._config.fps,
                )
            self._pipeline.start(cfg)
            log.info(
                "RealSense '%s' (SN:%s) started at %dx%d@%dfps",
                self._config.name,
                self._config.serial_number,
                self._config.width,
                self._config.height,
                self._config.fps,
            )
        except Exception as e:
            log.error(
                "Failed to start RealSense '%s' (SN:%s): %s",
                self._config.name,
                self._config.serial_number,
                e,
            )
            self._pipeline = None
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name=f"cam-{self._config.name}"
        )
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_depth(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._depth_frame.copy() if self._depth_frame is not None else None

    def _capture_loop(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                color = frames.get_color_frame()
                if color:
                    img = np.asanyarray(color.get_data())
                    with self._lock:
                        self._frame = img
                    self._frame_count += 1
                if self._config.enable_depth:
                    depth = frames.get_depth_frame()
                    if depth:
                        with self._lock:
                            self._depth_frame = np.asanyarray(depth.get_data())
            except Exception as e:
                if self._running:
                    log.warning("Frame capture error on '%s': %s", self._config.name, e)
                    time.sleep(0.01)


class OpenCVStream:
    """USB / V4L2 camera via OpenCV. Frames are stored as RGB uint8."""

    def __init__(self, config: CameraConfig):
        self._config = config
        self._cap = None
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self) -> bool:
        try:
            import cv2
        except ImportError:
            log.error("opencv-python is not installed (needed for type: opencv cameras)")
            return False

        device = self._config.device
        open_target: str | int = device
        if isinstance(device, str) and device.isdigit():
            open_target = int(device)
        elif isinstance(device, str):
            from pathlib import Path

            resolved = Path(device).resolve()
            if resolved.exists():
                open_target = str(resolved)
                log.info(
                    "Camera '%s' device %s → %s",
                    self.name,
                    device,
                    open_target,
                )
        try:
            self._cap = cv2.VideoCapture(open_target, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                self._cap = cv2.VideoCapture(open_target)
            if not self._cap.isOpened():
                log.error("Failed to open OpenCV camera '%s' device=%s", self.name, device)
                self._cap = None
                return False
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._config.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._config.height)
            self._cap.set(cv2.CAP_PROP_FPS, self._config.fps)
            # Reduce internal buffering so SyncBuffer sees fresher frames.
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as exc:
            log.error("OpenCV camera '%s' start failed: %s", self.name, exc)
            self._cap = None
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name=f"cam-opencv-{self.name}"
        )
        self._thread.start()
        log.info(
            "OpenCV camera '%s' opened device=%s target=%dx%d@%dfps",
            self.name,
            device,
            self._config.width,
            self._config.height,
            self._config.fps,
        )
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_depth(self) -> Optional[np.ndarray]:
        return None

    def _capture_loop(self):
        import cv2

        period = 1.0 / max(self._config.fps, 1)
        while self._running and self._cap is not None:
            ok, bgr = self._cap.read()
            if not ok or bgr is None:
                time.sleep(0.01)
                continue
            if bgr.shape[0] != self._config.height or bgr.shape[1] != self._config.width:
                bgr = cv2.resize(bgr, (self._config.width, self._config.height))
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with self._lock:
                self._frame = rgb
            self._frame_count += 1
            time.sleep(period)


class MockStream:
    """Hardware-free RGB source: gray canvas plus a moving stripe."""

    def __init__(self, config: CameraConfig):
        self._config = config
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self) -> bool:
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name=f"mock-cam-{self._config.name}"
        )
        self._thread.start()
        log.info("Mock camera '%s' started %dx%d", self.name, self._config.width, self._config.height)
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_depth(self) -> Optional[np.ndarray]:
        return None

    def _loop(self):
        period = 1.0 / max(self._config.fps, 1)
        while self._running:
            img = np.full(
                (self._config.height, self._config.width, 3), 40, dtype=np.uint8
            )
            col = (self._frame_count * 4) % max(self._config.width, 1)
            img[:, col : col + 8] = (200, 80, 40)
            with self._lock:
                self._frame = img
            self._frame_count += 1
            time.sleep(period)


class MultiCameraCapture:
    """Manages multiple camera streams (OpenCV, RealSense, or mock)."""

    def __init__(self, configs: list[CameraConfig], mock: bool = False):
        self._mock = mock
        self._streams = []
        for cfg in configs:
            backend = "mock" if mock else str(cfg.backend or "realsense").lower()
            if backend == "mock":
                self._streams.append(MockStream(cfg))
            elif backend == "opencv":
                self._streams.append(OpenCVStream(cfg))
            elif backend == "realsense":
                self._streams.append(RealSenseStream(cfg))
            else:
                raise ValueError(f"Unsupported camera backend {backend!r} for {cfg.name}")

    def start_all(self) -> dict[str, bool]:
        results = {}
        for stream in self._streams:
            results[stream.name] = stream.start()
        return results

    def stop_all(self):
        for stream in self._streams:
            stream.stop()

    def get_frames(self) -> dict[str, Optional[np.ndarray]]:
        return {s.name: s.get_frame() for s in self._streams}

    def get_frame(self, name: str) -> Optional[np.ndarray]:
        for s in self._streams:
            if s.name == name:
                return s.get_frame()
        return None

    @property
    def camera_names(self) -> list[str]:
        return [s.name for s in self._streams]


def cameras_from_config(cameras_cfg: dict, mock: bool = False) -> MultiCameraCapture:
    configs = []
    for name, raw in cameras_cfg.items():
        if not isinstance(raw, dict):
            continue
        enabled = bool(raw.get("enabled", True))
        backend = str(raw.get("type") or raw.get("backend") or "realsense").lower()
        serial = str(raw.get("serial_number", "")).strip()
        device = str(raw.get("device", "/dev/video0")).strip()
        if not mock and not enabled:
            log.info("Camera '%s' disabled in config, skip", name)
            continue
        if not mock and backend == "realsense" and not serial:
            log.warning("Camera '%s' has empty serial_number, skip", name)
            continue
        if not mock and backend == "opencv" and not device:
            log.warning("Camera '%s' has empty device, skip", name)
            continue
        configs.append(
            CameraConfig(
                name=name,
                backend=backend,
                serial_number=serial,
                device=device,
                width=int(raw.get("width", 640)),
                height=int(raw.get("height", 480)),
                fps=int(raw.get("fps", 30)),
                enable_depth=bool(raw.get("enable_depth", False)),
            )
        )
    if not configs:
        log.warning("No cameras configured")
    return MultiCameraCapture(configs, mock=mock)


def list_realsense_devices() -> list[dict]:
    import pyrealsense2 as rs

    ctx = rs.context()
    result = []
    for dev in ctx.query_devices():
        info = {
            "name": dev.get_info(rs.camera_info.name),
            "serial_number": dev.get_info(rs.camera_info.serial_number),
        }
        if dev.supports(rs.camera_info.usb_type_descriptor):
            info["usb_type"] = dev.get_info(rs.camera_info.usb_type_descriptor)
        result.append(info)
    return result
