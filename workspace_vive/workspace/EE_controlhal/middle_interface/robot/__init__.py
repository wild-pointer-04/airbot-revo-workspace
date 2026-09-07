from .base_robot_adapter import BaseRobotAdapter
from .dummy_robot_adapter import DummyRobotAdapter

__all__ = ["BaseRobotAdapter", "DummyRobotAdapter", "create_adapter"]


def create_adapter(config: dict) -> BaseRobotAdapter:
    robot_cfg = config.get("robot") or {}
    name = str(robot_cfg.get("adapter", "dummy")).lower()
    if name == "dummy":
        return DummyRobotAdapter.from_config(robot_cfg)
    if name in {"arx", "command", "command_layer"}:
        from robot.adapters.command_layer import CommandLayerAdapter

        return CommandLayerAdapter.from_config(robot_cfg)
    raise ValueError(
        f"Unknown robot adapter {name!r}. Use dummy or arx "
        "(Robot_control_interface). Default is dummy."
    )
