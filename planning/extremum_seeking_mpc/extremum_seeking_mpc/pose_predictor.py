from dataclasses import dataclass
from typing import Any, List, Sequence, Tuple
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from common_python.get_ros_parameter import get_ros_parameter
from .util import Position2d, Pose, Velocity, calculate_decelerated_velocity, first_scalar


@dataclass
class DynamicState:

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_velocity: float = 0.0
    yaw_rate: float = 0.0


class PosePredictor:

    def __init__(
        self,
        node: Node,
        horizon_times: List[float],
        seek_y_positions: np.ndarray,
        buffer_size: int,
    ):
        self.node = node

        self.init_parameters(node)
        self.init_connections(node, buffer_size)

        self.ego_current_velocity = Velocity(linear=0.0, angular=0.0)
        self.has_received_odometry = False

        self.horizon_times = np.asarray(horizon_times, dtype=float)

        if self.horizon_times.ndim != 1:
            raise ValueError("horizon_times must be a one-dimensional sequence.")
        if len(self.horizon_times) == 0:
            raise ValueError("horizon_times must contain at least one prediction time.")
        if np.any(self.horizon_times <= 0.0):
            raise ValueError("All horizon_times must be positive.")
        if np.any(np.diff(self.horizon_times) <= 0.0):
            raise ValueError("horizon_times must be strictly increasing.")

        self.horizon_durations = np.diff(self.horizon_times, prepend=0.0)
        self.horizon_length = len(self.horizon_times)

        self.seek_y_positions = np.asarray(seek_y_positions, dtype=float)

        if self.seek_y_positions.shape[0] != self.horizon_length:
            raise ValueError(
                "The number of seek-point sets must match the number of horizon steps."
            )

        self.last_predicted_yaws = np.zeros(self.horizon_length, dtype=float)

    def init_parameters(self, node: Node) -> None:

        self.curvature_radius_maximum = self._get_parameter_safe(
            node, "curvature_radius_maximum"
        )
        self.planned_speed = first_scalar(
            self._get_parameter_safe(node, "planned_speed"), "planned_speed"
        )
        self.deceleration_angle_maximum = self._get_parameter_safe(
            node, "velocity_control.deceleration_angle_maximum"
        )
        self.deceleration_gain = self._get_parameter_safe(
            node, "velocity_control.deceleration_gain"
        )

        def default(name, value):
            return self._get_parameter_or_default(node, name, value)

        self.integration_dt = default("prediction.integration_dt", 0.01)

        self.velocity_response_time = default("prediction.velocity_response_time", 0.5)
        self.yaw_rate_response_time = default("prediction.yaw_rate_response_time", 0.3)

        self.linear_viscous_resistance = default("prediction.linear_viscous_resistance", 0.0)
        self.yaw_viscous_resistance = default("prediction.yaw_viscous_resistance", 0.0)

        self.vehicle_mass = default("vehicle.mass", -1.0)
        self.vehicle_yaw_inertia = default("vehicle.yaw_inertia", -1.0)
        self.wheel_diameter = default("wheel.diameter", -1.0)
        self.wheel_tread = default("wheel.tread", -1.0)

        self.wheel_torque_per_amp = default("motor.wheel_torque_per_amp", -1.0)
        self.current_limit_amp = default("motor.current_limit_amp", 50.0)

        self._validate_physical_parameters()

    @staticmethod
    def _get_parameter_safe(node: Node, name: str) -> Any:

        if node.has_parameter(name):
            return node.get_parameter(name).value
        return get_ros_parameter(node, name)

    @staticmethod
    def _get_parameter_or_default(node: Node, name: str, default_value):

        if not node.has_parameter(name):
            node.declare_parameter(name, default_value)
        return node.get_parameter(name).value

    def _validate_physical_parameters(self) -> None:

        positive_parameters = {
            "prediction.integration_dt": self.integration_dt,
            "prediction.velocity_response_time": self.velocity_response_time,
            "prediction.yaw_rate_response_time": self.yaw_rate_response_time,
            "vehicle.mass": self.vehicle_mass,
            "vehicle.yaw_inertia": self.vehicle_yaw_inertia,
            "wheel.diameter": self.wheel_diameter,
            "wheel.tread": self.wheel_tread,
            "motor.wheel_torque_per_amp": self.wheel_torque_per_amp,
            "motor.current_limit_amp": self.current_limit_amp,
        }

        invalid = [
            name for name, value in positive_parameters.items() if float(value) <= 0.0
        ]

        if invalid:
            raise ValueError(
                f"The following parameters must be positive: {', '.join(invalid)}"
            )

        if self.curvature_radius_maximum <= 0.0:
            raise ValueError("curvature_radius_maximum must be positive.")

    def init_connections(self, node: Node, buffer_size: int) -> None:
        self.actual_speed_sub = node.create_subscription(
            Odometry, 'sub_odom', self.odometry_callback, buffer_size
        )

    def odometry_callback(self, odom_msg: Odometry) -> None:

        self.ego_current_velocity = Velocity(
            linear=float(odom_msg.twist.twist.linear.x),
            angular=float(odom_msg.twist.twist.angular.z),
        )
        self.has_received_odometry = True

    def _normalize_curvatures(self, curvatures: Sequence[float]) -> np.ndarray:

        curvature_array = np.asarray(curvatures, dtype=float).reshape(-1)

        if curvature_array.size == 0:
            curvature_array = np.zeros(self.horizon_length, dtype=float)

        if curvature_array.size < self.horizon_length:
            curvature_array = np.pad(
                curvature_array,
                (0, self.horizon_length - curvature_array.size),
                mode='edge',
            )

        return curvature_array[:self.horizon_length]

    def _effective_curvature(self, curvature: float) -> float:

        curvature_deadband = 1.0 / self.curvature_radius_maximum
        if abs(curvature) <= curvature_deadband:
            return 0.0
        return float(curvature)

    def _calculate_target_linear_velocity(
        self,
        curvature: float,
        current_velocity: float,
        interval_duration: float,
    ) -> float:

        return calculate_decelerated_velocity(
            base_velocity=self.planned_speed,
            eval_velocity=current_velocity,
            curvature=curvature,
            dt=interval_duration,
            deceleration_angle_maximum=self.deceleration_angle_maximum,
            deceleration_gain=self.deceleration_gain,
        )


    def _calculate_desired_accelerations(
        self,
        state: DynamicState,
        curvature: float,
        interval_duration: float,
    ) -> Tuple[float, float]:

        target_velocity = self._calculate_target_linear_velocity(
            curvature, state.linear_velocity, interval_duration
        )

        curvature_reference_velocity = (
            state.linear_velocity
            if abs(state.linear_velocity) >= abs(target_velocity)
            else target_velocity
        )
        target_yaw_rate = curvature_reference_velocity * curvature

        desired_linear_acceleration = (
            target_velocity - state.linear_velocity
        ) / self.velocity_response_time
        desired_yaw_acceleration = (
            target_yaw_rate - state.yaw_rate
        ) / self.yaw_rate_response_time

        return float(desired_linear_acceleration), float(desired_yaw_acceleration)

    def _desired_accelerations_to_currents(
        self,
        state: DynamicState,
        desired_linear_acceleration: float,
        desired_yaw_acceleration: float,
    ) -> Tuple[float, float]:

        wheel_radius = 0.5 * self.wheel_diameter

        common_current = (
            wheel_radius
            * (
                self.vehicle_mass * desired_linear_acceleration
                + self.linear_viscous_resistance * state.linear_velocity
            )
            / (2.0 * self.wheel_torque_per_amp)
        )

        differential_current = (
            wheel_radius
            * (
                self.vehicle_yaw_inertia * desired_yaw_acceleration
                + self.yaw_viscous_resistance * state.yaw_rate
            )
            / (self.wheel_tread * self.wheel_torque_per_amp)
        )

        current_right = common_current + differential_current
        current_left = common_current - differential_current

        max_abs_current = max(abs(current_left), abs(current_right))
        if max_abs_current > self.current_limit_amp:
            scale = self.current_limit_amp / max_abs_current
            current_left *= scale
            current_right *= scale

        return float(current_left), float(current_right)

    def _currents_to_actual_accelerations(
        self,
        state: DynamicState,
        current_left: float,
        current_right: float,
    ) -> Tuple[float, float]:

        wheel_radius = 0.5 * self.wheel_diameter

        torque_left = self.wheel_torque_per_amp * current_left
        torque_right = self.wheel_torque_per_amp * current_right

        drive_force = (torque_right + torque_left) / wheel_radius
        linear_resistance_force = self.linear_viscous_resistance * state.linear_velocity
        linear_acceleration = (
            drive_force - linear_resistance_force
        ) / self.vehicle_mass

        yaw_moment = (
            self.wheel_tread / (2.0 * wheel_radius) * (torque_right - torque_left)
        )
        yaw_resistance_moment = self.yaw_viscous_resistance * state.yaw_rate
        yaw_acceleration = (
            yaw_moment - yaw_resistance_moment
        ) / self.vehicle_yaw_inertia

        return float(linear_acceleration), float(yaw_acceleration)

    @staticmethod
    def _integrate_state(
        state: DynamicState,
        linear_acceleration: float,
        yaw_acceleration: float,
        dt: float,
    ) -> DynamicState:

        next_linear_velocity = state.linear_velocity + linear_acceleration * dt
        next_yaw_rate = state.yaw_rate + yaw_acceleration * dt

        middle_linear_velocity = 0.5 * (state.linear_velocity + next_linear_velocity)
        middle_yaw_rate = 0.5 * (state.yaw_rate + next_yaw_rate)

        next_yaw = state.yaw + middle_yaw_rate * dt
        middle_yaw = 0.5 * (state.yaw + next_yaw)

        next_x = state.x + middle_linear_velocity * np.cos(middle_yaw) * dt
        next_y = state.y + middle_linear_velocity * np.sin(middle_yaw) * dt

        return DynamicState(
            x=float(next_x),
            y=float(next_y),
            yaw=float(next_yaw),
            linear_velocity=float(next_linear_velocity),
            yaw_rate=float(next_yaw_rate),
        )

    def _simulate_interval(
        self,
        initial_state: DynamicState,
        curvature: float,
        interval_duration: float,
    ) -> DynamicState:

        if interval_duration <= 0.0:
            return DynamicState(**vars(initial_state))

        effective_curvature = self._effective_curvature(curvature)

        num_steps = max(1, int(np.ceil(interval_duration / self.integration_dt)))
        dt = interval_duration / num_steps

        state = DynamicState(**vars(initial_state))

        for _ in range(num_steps):
            desired_linear_acceleration, desired_yaw_acceleration = (
                self._calculate_desired_accelerations(
                    state, effective_curvature, interval_duration
                )
            )
            current_left, current_right = self._desired_accelerations_to_currents(
                state, desired_linear_acceleration, desired_yaw_acceleration
            )
            actual_linear_acceleration, actual_yaw_acceleration = (
                self._currents_to_actual_accelerations(
                    state, current_left, current_right
                )
            )
            state = self._integrate_state(
                state, actual_linear_acceleration, actual_yaw_acceleration, dt
            )

        return state

    def predict_relative_ego_positions(self, curvatures: np.ndarray) -> np.ndarray:

        curvature_array = self._normalize_curvatures(curvatures)

        state = DynamicState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_velocity=float(self.ego_current_velocity.linear),
            yaw_rate=float(self.ego_current_velocity.angular),
        )

        ego_positions = np.zeros((self.horizon_length, 2), dtype=float)
        predicted_states = []

        for horizon_idx, (curvature, horizon_duration) in enumerate(
            zip(curvature_array, self.horizon_durations)
        ):
            state = self._simulate_interval(
                state, float(curvature), float(horizon_duration)
            )
            ego_positions[horizon_idx] = [state.x, state.y]
            predicted_states.append(DynamicState(**vars(state)))

        self.last_predicted_states = predicted_states
        self.last_predicted_yaws = np.array(
            [s.yaw for s in predicted_states], dtype=float
        )

        return ego_positions

    def predict_pose(self, curvature: float, horizon_time: float) -> Pose:

        initial_state = DynamicState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_velocity=float(self.ego_current_velocity.linear),
            yaw_rate=float(self.ego_current_velocity.angular),
        )

        predicted_state = self._simulate_interval(
            initial_state, float(curvature), float(horizon_time)
        )

        return Pose(
            pos=Position2d(predicted_state.x, predicted_state.y),
            yaw=predicted_state.yaw,
        )

    @staticmethod
    def create_rotation_matrix(angle: float) -> np.ndarray:

        return np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]],
            dtype=float,
        )

    @staticmethod
    def rotate_position(position: Position2d, angle: float) -> np.ndarray:

        rotation_matrix = PosePredictor.create_rotation_matrix(angle)
        return rotation_matrix @ position.as_array()

    def predict_relative_seek_positions(self, ego_positions: np.ndarray) -> np.ndarray:

        del ego_positions

        num_seek_points = self.seek_y_positions.shape[1]
        relative_seek_positions = np.zeros(
            (self.horizon_length, 2, num_seek_points), dtype=float
        )

        for horizon_idx in range(self.horizon_length):
            lateral_offsets = np.asarray(
                self.seek_y_positions[horizon_idx], dtype=float
            )
            local_offsets = np.vstack(
                [np.zeros_like(lateral_offsets), lateral_offsets]
            )
            rotation_matrix = self.create_rotation_matrix(
                self.last_predicted_yaws[horizon_idx]
            )
            relative_seek_positions[horizon_idx] = rotation_matrix @ local_offsets

        return relative_seek_positions

    def predict_absolute_seek_positions(
        self,
        ego_positions: np.ndarray,
        relative_seek_positions: np.ndarray,
    ) -> np.ndarray:

        ego_positions_array = np.asarray(ego_positions, dtype=float)
        relative_array = np.asarray(relative_seek_positions, dtype=float)

        expected_position_shape = (self.horizon_length, 2)
        if ego_positions_array.shape != expected_position_shape:
            raise ValueError(
                f"ego_positions must have shape {expected_position_shape}, "
                f"but got {ego_positions_array.shape}."
            )

        if (
            relative_array.ndim != 3
            or relative_array.shape[0] != self.horizon_length
            or relative_array.shape[1] != 2
        ):
            raise ValueError(
                "relative_seek_positions must have shape "
                "(horizon_length, 2, num_seek_points)."
            )

        return ego_positions_array[:, :, np.newaxis] + relative_array