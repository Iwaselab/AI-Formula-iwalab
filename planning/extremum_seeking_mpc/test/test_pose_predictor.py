import numpy as np
import pytest
from unittest.mock import MagicMock
from extremum_seeking_mpc.pose_predictor import DynamicState, PosePredictor
from extremum_seeking_mpc.util import Pose, Position2d, Velocity


@pytest.fixture
def mock_node():
    node = MagicMock()
    node.has_parameter.return_value = False
    params = {
        "curvature_radius_maximum": 100.0,
        "planned_speed": 5.0,
        "velocity_control.deceleration_angle_maximum": 0.5,
        "velocity_control.deceleration_gain": 1.0,
        "prediction.integration_dt": 0.01,
        "prediction.velocity_response_time": 0.5,
        "prediction.yaw_rate_response_time": 0.3,
        "prediction.linear_viscous_resistance": 0.0,
        "prediction.yaw_viscous_resistance": 0.0,
        "vehicle.mass": 200.0,
        "vehicle.yaw_inertia": 50.0,
        "wheel.diameter": 0.3,
        "wheel.tread": 0.8,
        "motor.wheel_torque_per_amp": 0.5,
        "motor.current_limit_amp": 50.0,
    }
    node.get_parameter.side_effect = lambda name: MagicMock(value=params[name])
    return node


def test_dynamic_state_initialization():
    state = DynamicState()
    assert state.x == 0.0
    assert state.y == 0.0
    assert state.yaw == 0.0


def test_pose_predictor_initialization(mock_node):
    horizon_times = [0.5, 1.0, 1.5]
    seek_y_positions = np.zeros((3, 5))
    predictor = PosePredictor(mock_node, horizon_times, seek_y_positions, buffer_size=10)

    assert predictor.horizon_length == 3
    assert predictor.planned_speed == 5.0
