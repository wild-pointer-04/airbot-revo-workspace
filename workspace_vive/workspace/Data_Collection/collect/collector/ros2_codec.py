"""JSON payload on std_msgs/String for /arm/state."""

from __future__ import annotations

import json
from typing import Any

import numpy as np

ARM_TOPIC = "/arm/state"


def encode_state(data: dict[str, Any]) -> str:
    out: dict[str, Any] = {}
    for key, value in data.items():
        if hasattr(value, "tolist"):
            out[key] = value.tolist()
        elif isinstance(value, (np.integer, np.floating)):
            out[key] = value.item()
        else:
            out[key] = value
    return json.dumps(out, ensure_ascii=False)


def decode_state(text: str) -> dict[str, Any]:
    raw = json.loads(text)
    if not isinstance(raw, dict):
        raise ValueError("state payload must be a JSON object")
    out: dict[str, Any] = {}
    for key, value in raw.items():
        if key in ("host_monotonic_ns", "sequence_id"):
            if isinstance(value, (list, tuple)):
                out[key] = int(value[0])
            else:
                out[key] = int(value)
        elif key == "observation.host_monotonic_ns":
            arr = value if isinstance(value, (list, tuple)) else [value]
            out[key] = np.asarray(arr, dtype=np.int64)
        elif isinstance(value, (list, tuple)):
            out[key] = np.asarray(value, dtype=np.float32)
        else:
            out[key] = value
    return out
