"""Create an ArmController from YAML."""

from __future__ import annotations

from typing import Any, Mapping

from backends.dummy import DummyArmController
from interface import ArmController


def create_controller(config: Mapping[str, Any] | None) -> ArmController:
    cfg = dict(config or {})
    name = str(cfg.get("backend", "dummy")).lower()
    if name == "dummy":
        return DummyArmController.from_config(cfg)
    if name in {"arx", "arx_ros2"}:
        from backends.arx_ros2 import ArxRos2Controller

        return ArxRos2Controller.from_config(cfg)
    if name in {"airbot", "airbot_sdk"}:
        from backends.airbot_sdk import AirbotSdkController

        return AirbotSdkController.from_config(cfg)
    raise ValueError(
        f"Unknown arm backend {name!r}. Use dummy, airbot_sdk, or arx_ros2."
    )
