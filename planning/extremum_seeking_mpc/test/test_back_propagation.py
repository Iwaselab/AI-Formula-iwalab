import numpy as np
import pytest
from unittest.mock import MagicMock
from extremum_seeking_mpc.back_propagation import BackPropagation


@pytest.fixture
def mock_node():
    node = MagicMock()
    # Parameters for interpolation
    params = {
        "backpropagation.gain_function_u": [0.0, 0.5, 1.0],
        "backpropagation.gain_function_positive_y": [1.0, 0.8, 0.5],
        "backpropagation.gain_function_negative_y": [1.0, 1.2, 1.5],
    }
    node.get_parameter.side_effect = lambda name: MagicMock(value=params[name])
    return node


def test_apply_backpropagation_positive(mock_node):
    bp = BackPropagation(mock_node)
    result = bp.apply_backpropagation(forward_risk_in=0.5, backward_risk_in=2.0)
    # gain_function_positive_y at 0.5 is 0.8 => 2.0 * 0.8 = 1.6
    assert pytest.approx(result, rel=1e-3) == 1.6


def test_apply_backpropagation_negative(mock_node):
    bp = BackPropagation(mock_node)
    result = bp.apply_backpropagation(forward_risk_in=0.5, backward_risk_in=-2.0)
    # gain_function_negative_y at 0.5 is 1.2 => -2.0 * 1.2 = -2.4
    assert pytest.approx(result, rel=1e-3) == -2.4
