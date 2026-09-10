"""Collector: cameras + AIRBOT arm + Revo2 hand → LeRobot."""

from collector.collector import Collector
from collector.features import build_features

__all__ = ["Collector", "build_features"]
