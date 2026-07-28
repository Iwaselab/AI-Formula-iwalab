from typing import Tuple
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from common_python.get_ros_parameter import get_ros_parameter
from .object_risk_calculator import ObjectRiskCalculator
from .path_optimizer import PathOptimizer
from .pose_predictor import PosePredictor
from .road_risk_calculator import RoadRiskCalculator
from .util import Side, Vector2, first_scalar

class ExtremumSeekingMpc(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc')

        self.init_parameters()
        self.init_members()
        self.init_connections()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.curvatures = np.zeros(self.horizon_length, dtype=float)
        self.waiting_for_odometry_logged = False

        self.timer = self.create_timer(
            self.control_period,
            self.publish_cmd_vel_timer_callback,
        )

        self.get_logger().info(
            "ExtremumSeekingMpc started with dynamic "
            "torque/acceleration-aware pose prediction."
        )
        self.get_logger().info(
            f"planned_speed = {self.ego_target_velocity:.6f} m/s"
        )

    def init_parameters(self) -> None:

        def get(name, cast=float):
            return cast(get_ros_parameter(self, name))

        self.ego_target_velocity = first_scalar(
            get_ros_parameter(self, "planned_speed"), "planned_speed"
        )
        self.predict_horizon = np.asarray(
            get_ros_parameter(self, "horizon_times"), dtype=float
        ).reshape(-1)

        self.curvature_gain = get("curvature_gain")
        self.benefit_gain = get("benefit_gain")
        self.deceleration_angle_maximum = get(
            "velocity_control.deceleration_angle_maximum"
        )
        self.deceleration_gain = get("velocity_control.deceleration_gain")
        self.base_footprint_frame_id = get("base_footprint_frame_id", str)
        self.control_period = get("control_period")
        self.buffer_size = get("buffer_size", int)

        if self.predict_horizon.size == 0:
            raise ValueError("horizon_times must contain at least one value.")
        if np.any(self.predict_horizon <= 0.0):
            raise ValueError("All horizon_times must be positive.")
        if np.any(np.diff(self.predict_horizon) <= 0.0):
            raise ValueError("horizon_times must be strictly increasing.")
        if self.control_period <= 0.0:
            raise ValueError("control_period must be positive.")
        if self.buffer_size <= 0:
            raise ValueError("buffer_size must be positive.")
        if self.deceleration_angle_maximum < 0.0:
            raise ValueError(
                "velocity_control.deceleration_angle_maximum must be non-negative."
            )
        if self.deceleration_gain < 0.0:
            raise ValueError(
                "velocity_control.deceleration_gain must be non-negative."
            )

        self.horizon_length = len(self.predict_horizon)

    def init_members(self) -> None:

        self.object_risk_calculator = ObjectRiskCalculator(self, self.buffer_size)
        self.road_risk_calculator = RoadRiskCalculator(self, self.buffer_size)
        self.path_optimizer = PathOptimizer(self, self.control_period)

        init_seek_points_array = np.array(
            [c.seek_points for c in self.path_optimizer.extremum_seeking_controllers],
            dtype=float,
        )

        if len(init_seek_points_array) != self.horizon_length:
            raise ValueError(
                "The number of PathOptimizer controllers "
                "must match the number of horizon_times."
            )

        self.pose_predictor = PosePredictor(
            self,
            self.predict_horizon.tolist(),
            init_seek_points_array,
            self.buffer_size,
        )

    def init_connections(self) -> None:
        self.twist_pub = self.create_publisher(
            Twist, 'pub_twist_command', self.buffer_size
        )

    def calculate_effective_curvatures(
        self, curvatures: np.ndarray
    ) -> np.ndarray:
        curvature_array = np.asarray(curvatures, dtype=float).reshape(-1)

        if curvature_array.size != self.horizon_length:
            raise ValueError(
                "curvatures length must match the prediction horizon length."
            )

        return self.curvature_gain * curvature_array

    def predict_ego_position(
        self, effective_curvatures: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        ego_positions = self.pose_predictor.predict_relative_ego_positions(
            effective_curvatures
        )
        seek_positions_relative = self.pose_predictor.predict_relative_seek_positions(
            ego_positions
        )
        seek_positions = self.pose_predictor.predict_absolute_seek_positions(
            ego_positions, seek_positions_relative
        )

        return ego_positions, seek_positions

    def calculate_object_risk(self, seek_positions: np.ndarray) -> np.ndarray:
        return self.object_risk_calculator.compute_object_risk(seek_positions)

    def calculate_road_risk(
        self, seek_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

        left_road_risk, left_y_hat = self.road_risk_calculator.compute_road_risk(
            seek_positions, Side.LEFT
        )
        right_road_risk, right_y_hat = self.road_risk_calculator.compute_road_risk(
            seek_positions, Side.RIGHT
        )
        benefit = self.road_risk_calculator.get_benefit_value(
            seek_positions, left_y_hat, right_y_hat
        )

        return left_road_risk, right_road_risk, benefit

    def calculate_total_risk(
        self,
        object_risk: np.ndarray,
        left_road_risk: np.ndarray,
        right_road_risk: np.ndarray,
        benefit: np.ndarray,
    ) -> np.ndarray:

        return (
            np.asarray(object_risk, dtype=float)
            + np.asarray(left_road_risk, dtype=float)
            + np.asarray(right_road_risk, dtype=float)
            - self.benefit_gain * np.asarray(benefit, dtype=float)
        )

    def update_curvatures(self, total_risk: np.ndarray) -> np.ndarray:

        updated_curvatures = np.asarray(
            self.path_optimizer.apply_extremum_seeking_control(total_risk),
            dtype=float,
        ).reshape(-1)

        if updated_curvatures.size != self.horizon_length:
            raise ValueError(
                "PathOptimizer returned a curvature sequence whose length "
                "does not match horizon_times."
            )
        if not np.all(np.isfinite(updated_curvatures)):
            raise ValueError(
                "PathOptimizer returned NaN or Inf in the curvature sequence."
            )

        self.curvatures = updated_curvatures
        return self.curvatures.copy()

    def calculate_linear_velocity_reference(
        self, effective_curvature: float
    ) -> float:

        first_horizon_time = float(self.predict_horizon[0])

        estimated_yaw_angle = abs(
            self.ego_target_velocity * effective_curvature * first_horizon_time
        )
        excess_yaw_angle = estimated_yaw_angle - self.deceleration_angle_maximum

        if excess_yaw_angle <= 0.0:
            return self.ego_target_velocity

        target_speed_magnitude = max(
            abs(self.ego_target_velocity) - self.deceleration_gain * excess_yaw_angle,
            0.0,
        )
        target_direction = 1.0 if self.ego_target_velocity >= 0.0 else -1.0

        return target_direction * target_speed_magnitude

    @staticmethod
    def calculate_yaw_rate_reference(
        linear_velocity_reference: float, effective_curvature: float
    ) -> float:

        return float(linear_velocity_reference * effective_curvature)

    def calculate_control_reference(
        self, effective_curvatures: np.ndarray
    ) -> Tuple[float, float]:

        first_curvature = float(effective_curvatures[0])

        linear_velocity_reference = self.calculate_linear_velocity_reference(
            first_curvature
        )
        yaw_rate_reference = self.calculate_yaw_rate_reference(
            linear_velocity_reference, first_curvature
        )

        return linear_velocity_reference, yaw_rate_reference

    def publish_cmd_vel(
        self, vehicle_linear_velocity: float, yaw_rate: float
    ) -> None:

        twist_msg = Twist()
        twist_msg.linear.x = float(vehicle_linear_velocity)
        twist_msg.angular.z = float(yaw_rate)
        self.twist_pub.publish(twist_msg)

    def tf_viewer(self, ego_positions: np.ndarray) -> None:

        positions = np.asarray(ego_positions, dtype=float)
        predicted_yaws = np.asarray(
            self.pose_predictor.last_predicted_yaws, dtype=float
        )
        now = self.get_clock().now().to_msg()

        for horizon_idx, position in enumerate(positions):
            yaw = (
                float(predicted_yaws[horizon_idx])
                if horizon_idx < predicted_yaws.size
                else 0.0
            )

            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.base_footprint_frame_id
            transform.child_frame_id = f"predicted_ego_pose_{horizon_idx}"
            transform.transform.translation.x = float(position[Vector2.X])
            transform.transform.translation.y = float(position[Vector2.Y])
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = float(np.sin(0.5 * yaw))
            transform.transform.rotation.w = float(np.cos(0.5 * yaw))

            self.tf_broadcaster.sendTransform(transform)

    def publish_cmd_vel_timer_callback(self) -> None:

        if not self.pose_predictor.has_received_odometry:
            if not self.waiting_for_odometry_logged:
                self.get_logger().warning(
                    "Waiting for odometry. Publishing zero command."
                )
                self.waiting_for_odometry_logged = True
            self.publish_cmd_vel(0.0, 0.0)
            return

        self.waiting_for_odometry_logged = False

        try:
            current_effective_curvatures = self.calculate_effective_curvatures(
                self.curvatures
            )
            _evaluated_ego_positions, evaluated_seek_positions = (
                self.predict_ego_position(current_effective_curvatures)
            )

            object_risk = self.calculate_object_risk(evaluated_seek_positions)
            left_road_risk, right_road_risk, benefit = self.calculate_road_risk(
                evaluated_seek_positions
            )
            total_risk = self.calculate_total_risk(
                object_risk, left_road_risk, right_road_risk, benefit
            )

            updated_curvatures = self.update_curvatures(total_risk)
            updated_effective_curvatures = self.calculate_effective_curvatures(
                updated_curvatures
            )

            vehicle_linear_velocity, yaw_rate = self.calculate_control_reference(
                updated_effective_curvatures
            )

            raw_curvature = float(
                updated_curvatures[0]
            )
            
            effective_curvature = float(
                updated_effective_curvatures[0]
            )

            first_horizon_time = float(
                self.predict_horizon[0]
            )

            estimated_yaw_angle = abs(
                self.ego_target_velocity
    	        * effective_curvature
    	        * first_horizon_time
            )

            if abs(vehicle_linear_velocity) < 1.0e-9:
                self.get_logger().warning(
                    "V_ref became zero: "
                    f"planned_speed={self.ego_target_velocity:.6f}, "
                    f"raw_curvature={raw_curvature:.6f}, "
                    f"effective_curvature={effective_curvature:.6f}, "
                    f"horizon_time={first_horizon_time:.6f}, "
                    f"estimated_yaw_angle={estimated_yaw_angle:.6f}, "
                    f"angle_threshold="
                    f"{self.deceleration_angle_maximum:.6f}, "
                    f"deceleration_gain="
                    f"{self.deceleration_gain:.6f}"
                )

            commanded_ego_positions, _commanded_seek_positions = (
                self.predict_ego_position(updated_effective_curvatures)
            )

            self.publish_cmd_vel(vehicle_linear_velocity, yaw_rate)
            self.tf_viewer(commanded_ego_positions)

        except Exception as error:
            self.get_logger().error(
                f"Extremum Seeking control cycle failed: {error}"
            )
            self.publish_cmd_vel(0.0, 0.0)

    def stop_vehicle(self) -> None:

        self.publish_cmd_vel(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    extremum_seeking_mpc = ExtremumSeekingMpc()

    try:
        rclpy.spin(extremum_seeking_mpc)
    except KeyboardInterrupt:
        extremum_seeking_mpc.get_logger().info(
            "KeyboardInterrupt received. Publishing stop command."
        )
    finally:
        extremum_seeking_mpc.stop_vehicle()
        extremum_seeking_mpc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
