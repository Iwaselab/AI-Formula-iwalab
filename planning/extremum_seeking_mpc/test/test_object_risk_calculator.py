import numpy as np
import pytest
from unittest.mock import MagicMock
from extremum_seeking_mpc.object_risk_calculator import ObjectRiskCalculator


@pytest.fixture
def mock_node():
    node = MagicMock()
    params = {
        "object_risk_potential.variance_x": 0.8,
        "object_risk_potential.variance_y": 0.8,
        "object_risk_potential.correlation_coefficient": 0.0,
        "object_risk_potential.gain": 2.0,
        "object_risk_potential.obstacle_class_ids": [1, 2],
        "object_risk_potential.crosswalk_class_ids": [10],
        "object_risk_potential.crosswalk_gain": 1.0,
    }
    node.get_parameter.side_effect = lambda name: MagicMock(value=params[name])
    return node


def test_empty_object_risk(mock_node):
    calculator = ObjectRiskCalculator(mock_node, buffer_size=10)
    seek_positions = np.zeros((3, 2, 5))
    risks = calculator.compute_object_risk(seek_positions)
    assert risks.shape == (3, 5)
    assert np.all(risks == 0.0)


def test_get_object_risk_value(mock_node):
    calculator = ObjectRiskCalculator(mock_node, buffer_size=10)
    obj = MagicMock()
    obj.x = 5.0
    obj.y = 0.0
    obj.class_id = 1
    obj.width = 1.0
    obj.confidence = 1.0

    seek_positions = np.array([
        [5.0, 5.0, 5.0],
        [-1.0, 0.0, 1.0]
    ])

    risks = calculator.get_object_risk_value([obj], seek_positions)
    assert len(risks) == 3
    # Peak risk should be closest to (5.0, 0.0) at index 1
    assert np.argmax(risks) == 1
