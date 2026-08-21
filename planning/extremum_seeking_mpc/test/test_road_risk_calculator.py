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
        [[0.0, 0.0, 0.0, 0.0, 0.0], [-1.0, -0.5, 0.0, 0.5, 1.0]],
        [[1.0, 1.0, 1.0, 1.0, 1.0], [-1.0, -0.5, 0.0, 0.5, 1.0]],
    ])

    # Test scalar inputs
    benefits_scalar = calculator.get_benefit_value(seek_positions, y_hat_l=2.0, y_hat_r=-2.0)
    assert benefits_scalar.shape == (2, 5)
    assert np.argmax(benefits_scalar[0]) == 2

    # Test array inputs with different center per horizon
    y_hat_l_arr = np.array([2.0, 3.0])
    y_hat_r_arr = np.array([-2.0, -1.0])
    # Horizon 0 center: (2.0 + -2.0)/2 = 0.0 -> peak at seek_y = 0.0 (index 2)
    # Horizon 1 center: (3.0 + -1.0)/2 = 1.0 -> peak at seek_y = 1.0 (index 4)
    benefits_array = calculator.get_benefit_value(
        seek_positions, y_hat_l=y_hat_l_arr, y_hat_r=y_hat_r_arr
    )
    assert benefits_array.shape == (2, 5)
    assert np.argmax(benefits_array[0]) == 2
    assert np.argmax(benefits_array[1]) == 4


def test_compute_road_risk_returns_y_hats_array(mock_node):
    calculator = RoadRiskCalculator(mock_node, buffer_size=10)
    seek_positions = np.array([
        [[0.0, 0.0, 0.0, 0.0, 0.0], [-1.0, -0.5, 0.0, 0.5, 1.0]],
        [[1.0, 1.0, 1.0, 1.0, 1.0], [-1.0, -0.5, 0.0, 0.5, 1.0]],
    ])

    risks, y_hats = calculator.compute_road_risk(seek_positions, Side.LEFT)
    assert risks.shape == (2, 5)
    assert isinstance(y_hats, np.ndarray)
    assert len(y_hats) == 2

