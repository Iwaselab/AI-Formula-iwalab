from types import SimpleNamespace

import numpy as np

from extremum_seeking_mpc.extremum_seeking_mpc import ExtremumSeekingMpc


def make_feedback_node(enabled=True, gain=2.0):
    node = ExtremumSeekingMpc.__new__(ExtremumSeekingMpc)
    node.rear_angle_feedback_enabled = enabled
    node.rear_angle_gain = gain
    node.rear_axle_distance_m = 1.0
    node.rear_angle_sign = -1.0
    node.rear_angle_offset_rad = 0.0
    node.rear_angle_deadband_rad = 0.01
    node.rear_angle_max_correction_rad_s = 0.05
    node.rear_angle_timeout_sec = 0.5
    node._rear_angle_received = True
    node._rear_angle_raw = 0.0
    node._rear_angle_receive_time_ns = 1_000_000_000
    node._rear_angle_clock_rollback = False
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_100_000_000)
    )
    return node


def test_disabled_feedback_matches_existing_ff():
    node = make_feedback_node(enabled=False)
    correction, debug = node._calculate_rear_angle_correction(0.1)

    assert correction == 0.0
    assert not debug["value_valid"]


def test_zero_gain_matches_existing_ff():
    node = make_feedback_node(gain=0.0)
    correction, debug = node._calculate_rear_angle_correction(0.1)

    assert correction == 0.0
    assert not debug["value_valid"]


def test_positive_and_negative_curvature_targets_reverse():
    node = make_feedback_node()
    node._rear_angle_raw = 0.0

    positive_correction, positive_debug = node._calculate_rear_angle_correction(
        0.2
    )
    node._rear_angle_raw = 0.0
    negative_correction, negative_debug = node._calculate_rear_angle_correction(
        -0.2
    )

    assert positive_debug["target_rear_angle_rad"] < 0.0
    assert negative_debug["target_rear_angle_rad"] > 0.0
    assert positive_correction != negative_correction


def test_zero_curvature_target_is_zero():
    node = make_feedback_node()
    correction, debug = node._calculate_rear_angle_correction(0.0)

    assert correction == 0.0
    assert debug["target_rear_angle_rad"] == 0.0


def test_deadband_and_correction_limit():
    node = make_feedback_node(gain=100.0)
    node.rear_angle_deadband_rad = 0.01

    node._rear_angle_raw = 1.0
    correction, debug = node._calculate_rear_angle_correction(0.0)

    assert debug["value_valid"]
    assert correction == -0.05

    node.rear_angle_deadband_rad = 1.0
    correction, debug = node._calculate_rear_angle_correction(0.0)
    assert correction == 0.0


def test_unreceived_expired_nan_and_clock_rollback_disable_correction():
    node = make_feedback_node()

    node._rear_angle_received = False
    correction, _ = node._calculate_rear_angle_correction(0.1)
    assert correction == 0.0

    node._rear_angle_received = True
    node._rear_angle_receive_time_ns = 500_000_000
    correction, debug = node._calculate_rear_angle_correction(0.1)
    assert correction == 0.0
    assert not debug["value_valid"]

    node._rear_angle_receive_time_ns = 1_000_000_000
    node._rear_angle_raw = np.nan
    correction, debug = node._calculate_rear_angle_correction(0.1)
    assert correction == 0.0
    assert not debug["value_valid"]

    node._rear_angle_raw = 0.0
    node._rear_angle_clock_rollback = True
    correction, debug = node._calculate_rear_angle_correction(0.1)
    assert correction == 0.0
    assert not debug["value_valid"]
