import numpy as np
import pytest
from extremum_seeking_mpc.extremum_seeking_controller import ExtremumSeekingController
from extremum_seeking_mpc.util import ControllerParameters, LowPassFilterParameters


@pytest.fixture
def controller():
    params = ControllerParameters(
        seek_gain=1.0,
        seek_amp=0.5,
        curvature_max=0.3,
        curvature_min=-0.3,
        feedback_gain=0.1,
        sin_period=1.0,
    )
    lowpass_params = LowPassFilterParameters(A=0.9, B=0.1, C=1.0)
    control_period = 0.1
    return ExtremumSeekingController(params, lowpass_params, control_period)


def test_controller_initialization(controller):
    assert len(controller.seek_points) == 5
    assert controller.curvature_max == 0.3
    assert controller.curvature_min == -0.3


def test_apply_risk_moving_average(controller):
    risk_in = np.array([0.1, 0.2, 0.0, 0.2, 0.1])
    moving_avg = controller.apply_risk_moving_average(risk_in)
    assert isinstance(moving_avg, float)


def test_optimize_input_clipping(controller):
    # Test positive upper bound clip
    out_high = controller.optimize_input(moving_average=1.0, backpropagation_value=1.0)
    assert out_high == 0.3

    # Test negative lower bound clip
    out_low = controller.optimize_input(moving_average=-2.0, backpropagation_value=-2.0)
    assert out_low == -0.3

    # Test within bounds
    out_normal = controller.optimize_input(moving_average=0.05, backpropagation_value=0.0)
    assert -0.3 <= out_normal <= 0.3
