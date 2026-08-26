from types import SimpleNamespace

from extremum_seeking_mpc.extremum_seeking_mpc import ExtremumSeekingMpc, State


def test_traffic_sign_scale_decels_before_stop():
    node = ExtremumSeekingMpc.__new__(ExtremumSeekingMpc)
    node._latest_rects = [
        SimpleNamespace(class_id=5, width=20.0),
        SimpleNamespace(class_id=6, width=10.0),
    ]
    node._state = State.RUNNING
    node._stop_sign_class_id = 5
    node._go_sign_class_id = 6
    node._stop_bbox_width_threshold = 45.0
    node._decel_bbox_width_start = 15.0
    node._min_velocity_scale = 0.2

    assert node._compute_traffic_sign_velocity_scale() < 1.0
    assert node._compute_traffic_sign_velocity_scale() > 0.0


def test_stop_light_zeroes_velocity_when_stopped():
    node = ExtremumSeekingMpc.__new__(ExtremumSeekingMpc)
    node._latest_rects = [SimpleNamespace(class_id=5, width=50.0)]
    node._state = State.STOPPED
    node._stop_sign_class_id = 5
    node._go_sign_class_id = 6
    node._stop_bbox_width_threshold = 45.0
    node._decel_bbox_width_start = 15.0
    node._min_velocity_scale = 0.2

    assert node._compute_traffic_sign_velocity_scale() == 0.0
