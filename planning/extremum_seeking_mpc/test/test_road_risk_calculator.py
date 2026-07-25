import numpy as np
import pytest
from unittest.mock import MagicMock
from extremum_seeking_mpc.road_risk_calculator import RoadRiskCalculator
from extremum_seeking_mpc.util import Side


@pytest.fixture
def mock_node():
    node = MagicMock()
    params = {
        "road_risk_potential.left_gradient": 1.0,
        "road_risk_potential.right_gradient": 1.0,
        "road_risk_potential.margin": 0.2,
        "road_risk_potential.offset": 1.0,
        "road_risk_potential.gain": 1.0,
        "road_weight.u": [0.0, 0.25, 0.5, 0.75, 1.0],
        "road_weight.num_function": 5,
        "road_weight.num_function_coefficients": 3,
        "road_identification.max_point_length": 50.0,
        "road_identification.identification_gain": 0.01,
        "road_identification.forget_vector": [1.0, 1.0, 1.0],
        "road_identification.road_parameter_limit": 1.0,
        "road_benefit_function.scale": 1.0,
        "road_benefit_function.covariance": 0.5,
    }
    node.get_parameter.side_effect = lambda name: MagicMock(value=params[name])
    return node


def test_get_road_risk_value(mock_node):
    calculator = RoadRiskCalculator(mock_node, buffer_size=10)
    seek_y = np.array([-1.0, 0.0, 1.0])
    y_hat = 0.0

    risk_left = calculator.get_road_risk_value(seek_y, y_hat, Side.LEFT)
    risk_right = calculator.get_road_risk_value(seek_y, y_hat, Side.RIGHT)

    assert len(risk_left) == 3
    assert len(risk_right) == 3


def test_get_benefit_value(mock_node):
    calculator = RoadRiskCalculator(mock_node, buffer_size=10)
    seek_positions = np.array([
        [[0.0, 0.0, 0.0, 0.0, 0.0], [-1.0, -0.5, 0.0, 0.5, 1.0]]
    ])

    benefits = calculator.get_benefit_value(seek_positions, y_hat_l=2.0, y_hat_r=-2.0)
    # Center y_hat is (2.0 + -2.0) / 2 = 0.0
    # Peak of benefit should be at seek_y = 0.0 (index 2)
    assert np.argmax(benefits[0]) == 2
