import numpy as np

from rclpy.node import Node

from .back_propagation import BackPropagation
from common_python.get_ros_parameter import get_ros_parameter
from .extremum_seeking_controller import ExtremumSeekingController
from .util import ControllerParameters, LowPassFilterParameters


class PathOptimizer:
    def __init__(self, node: Node, control_period: float):
        seek_gain = get_ros_parameter(node, "controller_parameters.seek_gain")
        seek_amp = get_ros_parameter(node, "controller_parameters.seek_amp")
        curvature_limit = get_ros_parameter(node, "controller_parameters.curvature_limit")
        feedback_gain = get_ros_parameter(node, "controller_parameters.feedback_gain")
        sin_period = get_ros_parameter(node, "controller_parameters.sin_period")

        mpc_params = ControllerParameters(
            seek_gain=seek_gain,
            seek_amp=seek_amp,
            curvature_max=curvature_limit,
            curvature_min=-curvature_limit,
            feedback_gain=feedback_gain,
            sin_period=sin_period
        )

        lowpass_A = get_ros_parameter(node, "lowpassfilter.A")
        lowpass_B = get_ros_parameter(node, "lowpassfilter.B")
        lowpass_C = get_ros_parameter(node, "lowpassfilter.C")

        lowpass_params = LowPassFilterParameters(
            A=lowpass_A, B=lowpass_B, C=lowpass_C
        )

        horizon_times = get_ros_parameter(node, "horizon_times")
        if not isinstance(horizon_times, (list, tuple)) or len(horizon_times) == 0:
            raise ValueError("horizon_times must contain at least one value.")

        self.extremum_seeking_controllers = [
            ExtremumSeekingController(mpc_params, lowpass_params, control_period)
            for _ in range(len(horizon_times))
        ]

        self.backpropagation = BackPropagation(node)

    def apply_extremum_seeking_control(self, risk: np.ndarray) -> np.ndarray:
        controllers = self.extremum_seeking_controllers

        # Calculate moving averages
        risk_moving_averages = [
            controller.apply_risk_moving_average(r) for controller, r in zip(controllers, risk)
        ]

        # Calculate backpropagation values for every horizon point.
        curvatures = []
        for controller_idx, (controller, risk_moving_average) in enumerate(
            zip(controllers, risk_moving_averages)
        ):
            backpropagation_value = sum(
                self.backpropagation.apply_backpropagation(
                    risk_moving_average,
                    risk_moving_averages[backward_idx],
                )
                for backward_idx in range(controller_idx + 1, len(risk_moving_averages))
            )
            curvatures.append(
                controller.optimize_input(risk_moving_average, backpropagation_value)
            )

        return np.array(curvatures)
