"""Protocol parse tests. Bad packets must be dropped, never crash."""

from __future__ import annotations

import json
import math

from protocol.pico_packet import PicoTeleopPacket


def _valid_dict(**overrides):
    payload = {
        "protocol": "pico_teleop",
        "version": 1,
        "seq": 10,
        "device_time_ns": 123,
        "wall_time_unix_ms": 456,
        "hmd": {
            "valid": True,
            "position": [0.0, 1.6, 0.0],
            "rotation": [0.0, 0.0, 0.0, 1.0],
        },
        "left_controller": {
            "valid": True,
            "position": [0.1, 1.2, 0.3],
            "rotation": [0.0, 0.0, 0.0, 1.0],
            "linear_velocity": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "trigger": 0.0,
            "grip": 0.0,
            "primary_button": False,
            "secondary_button": False,
            "joystick": [0.0, 0.0],
        },
        "right_controller": {
            "valid": True,
            "position": [0.2, 1.1, 0.4],
            "rotation": [0.0, 0.0, 0.0, 1.0],
            "linear_velocity": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "trigger": 0.2,
            "grip": 0.8,
            "primary_button": False,
            "secondary_button": False,
            "joystick": [0.1, -0.2],
        },
    }
    payload.update(overrides)
    return payload


def test_valid_packet_roundtrip():
    packet = PicoTeleopPacket.from_dict(_valid_dict())
    assert packet.seq == 10
    assert packet.right_controller.grip == 0.8
    again = PicoTeleopPacket.from_json_bytes(packet.to_json().encode("utf-8"))
    assert again is not None
    assert again.seq == 10


def test_nan_is_dropped():
    data = _valid_dict()
    data["right_controller"]["position"] = [math.nan, 0.0, 0.0]
    assert PicoTeleopPacket.from_json_bytes(json.dumps(data).encode()) is None


def test_inf_is_dropped():
    data = _valid_dict()
    data["hmd"]["rotation"] = [0.0, 0.0, 0.0, math.inf]
    assert PicoTeleopPacket.from_json_bytes(json.dumps(data).encode()) is None


def test_missing_field_is_dropped():
    data = _valid_dict()
    del data["right_controller"]
    assert PicoTeleopPacket.from_json_bytes(json.dumps(data).encode()) is None


def test_wrong_protocol_version_is_dropped():
    data = _valid_dict(protocol="pico_teleop", version=99)
    assert PicoTeleopPacket.from_json_bytes(json.dumps(data).encode()) is None
    data = _valid_dict(protocol="other")
    assert PicoTeleopPacket.from_json_bytes(json.dumps(data).encode()) is None


def test_wrong_vector_length_is_dropped():
    data = _valid_dict()
    data["left_controller"]["position"] = [1.0, 2.0]
    assert PicoTeleopPacket.from_json_bytes(json.dumps(data).encode()) is None


def test_garbage_bytes_are_dropped():
    assert PicoTeleopPacket.from_json_bytes(b"not-json") is None
    assert PicoTeleopPacket.from_json_bytes(b"\xff\xfe") is None
