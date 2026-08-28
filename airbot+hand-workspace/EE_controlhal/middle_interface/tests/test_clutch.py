"""Clutch rising-edge reference reset and delayed VR baseline tests."""

from __future__ import annotations

import numpy as np

from processing.clutch import Clutch
from protocol.pico_packet import Pose


def _pose(x, y, z, qw=1.0):
    return Pose(
        position=np.array([x, y, z], dtype=np.float64),
        rotation=np.array([0.0, 0.0, 0.0, qw], dtype=np.float64),
    )


def test_press_move_release_press_again():
    clutch = Clutch(
        press_threshold=0.5,
        release_threshold=0.3,
        position_scale=1.0,
        delay_vr_baseline=False,
    )
    robot = _pose(0.3, 0.0, 0.2)

    r1 = clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    assert r1.pressed_edge and r1.active
    np.testing.assert_allclose(r1.target.position, robot.position)

    r2 = clutch.update(0.8, _pose(0.10, 0.0, 0.0), robot)
    assert r2.active and not r2.pressed_edge
    np.testing.assert_allclose(r2.target.position, [0.40, 0.0, 0.2])

    r3 = clutch.update(0.2, _pose(0.10, 0.0, 0.0), r2.target)
    assert r3.released_edge and not r3.active
    assert r3.target is None

    robot_held = r2.target
    r4 = clutch.update(0.8, _pose(1.0, 0.0, 0.0), robot_held)
    assert r4.pressed_edge and r4.active
    np.testing.assert_allclose(r4.target.position, robot_held.position)

    r5 = clutch.update(0.8, _pose(1.05, 0.0, 0.0), robot_held)
    np.testing.assert_allclose(
        r5.target.position, robot_held.position + np.array([0.05, 0.0, 0.0])
    )


def test_hysteresis_does_not_retrigger():
    clutch = Clutch(0.5, 0.3, 1.0, delay_vr_baseline=False)
    robot = _pose(0, 0, 0)
    assert not clutch.update(0.4, _pose(0, 0, 0), robot).active
    assert clutch.update(0.5, _pose(0, 0, 0), robot).pressed_edge
    mid = clutch.update(0.4, _pose(0, 0, 0), robot)
    assert mid.active and not mid.pressed_edge and not mid.released_edge
    assert clutch.update(0.3, _pose(0, 0, 0), robot).released_edge
    assert not clutch.update(0.4, _pose(0, 0, 0), robot).active


def test_position_scale():
    clutch = Clutch(0.5, 0.3, position_scale=0.5, delay_vr_baseline=False)
    robot = _pose(0.0, 0.0, 0.0)
    clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    moved = clutch.update(0.8, _pose(0.20, 0.0, 0.0), robot)
    np.testing.assert_allclose(moved.target.position, [0.10, 0.0, 0.0])


def test_delay_vr_baseline_skips_press_frame():
    clutch = Clutch(0.5, 0.3, 1.0, delay_vr_baseline=True)
    robot = _pose(0.3, 0.0, 0.2)

    press = clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    assert press.pressed_edge and press.active
    assert not press.vr_baseline_ready
    np.testing.assert_allclose(press.target.position, robot.position)

    capture = clutch.update(0.8, _pose(0.10, 0.0, 0.0), robot)
    assert capture.vr_baseline_ready
    np.testing.assert_allclose(capture.target.position, robot.position)

    moved = clutch.update(0.8, _pose(0.15, 0.0, 0.0), robot)
    np.testing.assert_allclose(moved.target.position, [0.35, 0.0, 0.2])


def test_delta_position_clamp():
    clutch = Clutch(
        0.5,
        0.3,
        1.0,
        max_delta_position=0.30,
        delay_vr_baseline=False,
    )
    robot = _pose(0.0, 0.0, 0.0)
    clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    jumped = clutch.update(0.8, _pose(1.0, 0.0, 0.0), robot)
    np.testing.assert_allclose(jumped.target.position, [0.30, 0.0, 0.0])


def test_zero_limits_disable_per_clutch_clamps():
    clutch = Clutch(
        0.5,
        0.3,
        position_scale=1.0,
        max_delta_position=0.0,
        max_delta_angle=0.0,
        delay_vr_baseline=False,
    )
    robot = _pose(0.0, 0.0, 0.0)
    clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    moved = clutch.update(0.8, _pose(1.0, 0.0, 0.0), robot)

    np.testing.assert_allclose(moved.delta_pos, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(moved.target.position, [1.0, 0.0, 0.0])


def test_rotation_scale_scales_delta_angle():
    import math

    from processing.math_pose import quat_angle

    yaw90 = np.array(
        [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)], dtype=np.float64
    )
    clutch = Clutch(
        0.5,
        0.3,
        position_scale=1.0,
        max_delta_angle=0.0,
        delay_vr_baseline=False,
        rotation_scale=0.5,
    )
    robot = _pose(0.0, 0.0, 0.0)
    clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    moved_hand = _pose(0.0, 0.0, 0.0)
    moved_hand.rotation = yaw90
    out = clutch.update(0.8, moved_hand, robot)
    np.testing.assert_allclose(quat_angle(out.target.rotation), math.pi / 4, atol=1e-5)


def test_rotation_scale_zero_holds_robot_orientation():
    import math

    from processing.math_pose import quat_angle

    yaw90 = np.array(
        [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)], dtype=np.float64
    )
    clutch = Clutch(
        0.5,
        0.3,
        delay_vr_baseline=False,
        rotation_scale=0.0,
        max_delta_angle=0.0,
    )
    robot = _pose(0.0, 0.0, 0.0)
    clutch.update(0.8, _pose(0.0, 0.0, 0.0), robot)
    moved_hand = _pose(0.10, 0.0, 0.0)
    moved_hand.rotation = yaw90
    out = clutch.update(0.8, moved_hand, robot)
    np.testing.assert_allclose(out.target.position, [0.10, 0.0, 0.0])
    np.testing.assert_allclose(quat_angle(out.target.rotation), 0.0, atol=1e-8)
