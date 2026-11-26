from enum import Enum, auto
from functools import partial
import threading
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster

from aiformula_interfaces.msg import ObjectInfo, ObjectInfoMultiArray, Rect, RectMultiArray
from common_python.get_ros_parameter import get_ros_parameter
from .lane_center_estimator import LaneCenterEstimator
from .sampling_planner import SamplingPlanner, SamplingPlannerConfig
from .util import Side, calculate_decelerated_velocity, first_scalar


class State(Enum):
    RUNNING = auto()
    DECELERATING = auto()
    STOPPED = auto()


class ExtremumSeekingMpc(Node):
    """Sampling-based MPC node using two-segment constant-curvature arc rollouts."""

    def __init__(self) -> None:
        super().__init__('extremum_seeking_mpc')

        self._lock = threading.Lock()
        self.init_parameters()
        self.init_members()
        self.init_connections()
        self._init_traffic_sign_parameters()
        self._init_traffic_sign_state()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_curvature = 0.0
        self.has_received_odometry = False
        self.waiting_for_odometry_logged = False
        self._latest_objects: List[ObjectInfo] = []
        self._last_cmd_yaw_rate = 0.0
        self._odom_yaw_rate = 0.0
        self._log_counter = 0

        self.timer = self.create_timer(
            self.control_period,
            self.publish_cmd_vel_timer_callback,
        )

        self.get_logger().info(
            f"Sampling MPC started: planned_speed={self.ego_target_velocity:.3f} m/s, "
            f"candidates={self.planner.config.num_candidates}^2, "
            f"curvature_limit={self.planner.config.curvature_limit:.4f} rad/m"
        )

    def init_parameters(self) -> None:
        def get(name: str, cast=float):
            return cast(get_ros_parameter(self, name))

        self.ego_target_velocity = first_scalar(
            get_ros_parameter(self, "planned_speed"), "planned_speed"
        )
        self.control_period = get("control_period")
        self.buffer_size = get("buffer_size", int)
        self.base_footprint_frame_id = get("base_footprint_frame_id", str)
        self.yaw_rate_gain = get("yaw_rate_gain")

        self.deceleration_angle_maximum = get(
            "velocity_control.deceleration_angle_maximum"
        )
        self.deceleration_gain = get("velocity_control.deceleration_gain")
        self.deceleration_dt = get("velocity_control.eval_dt")

        self.lane_max_point_length = get("lane.max_point_length")
        self.lane_width = get("lane.width")
        self.lane_min_segment_length = get("lane.min_segment_length")
        self.lane_max_gap = get("lane.max_gap")

        self.planner_config = SamplingPlannerConfig(
            curvature_limit=get("sampling.curvature_limit"),
            num_candidates=get("sampling.num_candidates", int),
            horizon_min_distance=get("sampling.horizon_min_distance"),
            horizon_max_distance=get("sampling.horizon_max_distance"),
            num_horizon_points=get("sampling.num_horizon_points", int),
            switch_distance=get("sampling.switch_distance"),
            far_weight=get("sampling.far_weight"),
            min_valid_points=get("sampling.min_valid_points", int),
            center_weight=get("sampling.center_weight"),
            boundary_weight=get("sampling.boundary_weight"),
            road_margin=get("sampling.road_margin"),
            smoothness_weight=get("sampling.smoothness_weight"),
            invalid_penalty=get("sampling.invalid_penalty"),
            max_total_yaw=get("sampling.max_total_yaw"),
            object_variance_x=get("object_risk_potential.variance_x"),
            object_variance_y=get("object_risk_potential.variance_y"),
            object_gain=get("object_risk_potential.gain"),
            obstacle_class_ids=tuple(
                int(c)
                for c in get_ros_parameter(self, "object_risk_potential.obstacle_class_ids")
            ),
        )

        if self.control_period <= 0.0:
            raise ValueError("control_period must be positive.")
        if self.buffer_size <= 0:
            raise ValueError("buffer_size must be positive.")
        if self.yaw_rate_gain < 0.0:
            raise ValueError("yaw_rate_gain must be non-negative.")

    def init_members(self) -> None:
        self.lane_estimator = LaneCenterEstimator(
            max_point_length=self.lane_max_point_length,
            lane_width=self.lane_width,
            min_segment_length=self.lane_min_segment_length,
            max_gap=self.lane_max_gap,
        )
        self.planner = SamplingPlanner(self.planner_config)

    def init_connections(self) -> None:
        self.twist_pub = self.create_publisher(
            Twist, 'pub_twist_command', self.buffer_size
        )
        self._bbox_sub = self.create_subscription(
            RectMultiArray, 'sub_bbox', self._bbox_callback, self.buffer_size
        )
        self._left_lane_sub = self.create_subscription(
            PointCloud2, 'sub_road_l',
            partial(self._lane_callback, side=Side.LEFT), self.buffer_size,
        )
        self._right_lane_sub = self.create_subscription(
            PointCloud2, 'sub_road_r',
            partial(self._lane_callback, side=Side.RIGHT), self.buffer_size,
        )
        self._odom_sub = self.create_subscription(
            Odometry, 'sub_odom', self._odom_callback, self.buffer_size
        )
        self._object_sub = self.create_subscription(
            ObjectInfoMultiArray, 'sub_object_info', self._object_callback, self.buffer_size,
        )

    def _lane_callback(self, msg: PointCloud2, side: Side) -> None:
        pc_iter = pc2.read_points(msg, field_names=['x', 'y'], skip_nans=True)
        lane_points = np.array(list(pc_iter), dtype=float)
        with self._lock:
            self.lane_estimator.update_lane(side, lane_points)

    def _odom_callback(self, msg: Odometry) -> None:
        self.has_received_odometry = True
        self._odom_yaw_rate = float(msg.twist.twist.angular.z)

    def _object_callback(self, msg: ObjectInfoMultiArray) -> None:
        with self._lock:
            self._latest_objects = list(msg.objects)

    def _init_traffic_sign_parameters(self) -> None:
        self._stop_sign_class_id = int(get_ros_parameter(self, 'stop_sign_class_id'))
        self._go_sign_class_id = int(get_ros_parameter(self, 'go_sign_class_id'))
        self._stop_bbox_width_threshold = float(
            get_ros_parameter(self, 'stop_bbox_width_threshold')
        )
        self._decel_bbox_width_start = float(
            get_ros_parameter(self, 'decel_bbox_width_start')
        )
        self._min_velocity_scale = float(get_ros_parameter(self, 'min_velocity_scale'))

        if not 0.0 <= self._min_velocity_scale <= 1.0:
            raise ValueError('min_velocity_scale must be in [0.0, 1.0]')
        if self._decel_bbox_width_start >= self._stop_bbox_width_threshold:
            raise ValueError(
                'decel_bbox_width_start must be smaller than stop_bbox_width_threshold'
            )

    def _init_traffic_sign_state(self) -> None:
        self._state: State = State.RUNNING
        self._latest_rects: list = []
        self._last_velocity_scale: float = 1.0

    def _bbox_callback(self, msg: RectMultiArray) -> None:
        with self._lock:
            self._latest_rects = list(msg.rects)
            self._update_traffic_sign_state()

    def _get_largest_bbox(self, class_id: int) -> Optional[Rect]:
        candidates = [r for r in self._latest_rects if r.class_id == class_id]
        if not candidates:
            return None
        return max(candidates, key=lambda r: r.width)

    def _update_traffic_sign_state(self) -> None:
        stop_sign_bbox = self._get_largest_bbox(self._stop_sign_class_id)
        go_sign_bbox = self._get_largest_bbox(self._go_sign_class_id)

        if self._state == State.RUNNING and stop_sign_bbox is not None:
            self._state = State.DECELERATING
            self.get_logger().info(
                f'[RUNNING -> DECELERATING] stop_sign detected '
                f'(bbox_width={stop_sign_bbox.width:.1f}px)'
            )
        elif self._state == State.DECELERATING:
            if go_sign_bbox is not None:
                self._state = State.RUNNING
                self.get_logger().info('[DECELERATING -> RUNNING] go_sign detected')
            elif (
                stop_sign_bbox is not None
                and stop_sign_bbox.width >= self._stop_bbox_width_threshold
            ):
                self._state = State.STOPPED
                self.get_logger().info(
                    f'[DECELERATING -> STOPPED] bbox_width={stop_sign_bbox.width:.1f}px '
                    f'>= threshold={self._stop_bbox_width_threshold:.1f}px'
                )
        elif self._state == State.STOPPED and go_sign_bbox is not None:
            self._state = State.RUNNING
            self.get_logger().info('[STOPPED -> RUNNING] go_sign detected')

    def _compute_traffic_sign_velocity_scale(self) -> float:
        if self._state == State.RUNNING:
            self._last_velocity_scale = 1.0
            return 1.0
        if self._state == State.STOPPED:
            self._last_velocity_scale = 0.0
            return 0.0

        stop_sign_bbox = self._get_largest_bbox(self._stop_sign_class_id)
        if stop_sign_bbox is None:
            return self._last_velocity_scale

        bbox_width = stop_sign_bbox.width
        span = self._stop_bbox_width_threshold - self._decel_bbox_width_start
        if span <= 0.0:
            self._last_velocity_scale = self._min_velocity_scale
            return self._last_velocity_scale

        progress = max(0.0, bbox_width - self._decel_bbox_width_start) / span
        progress = min(progress, 1.0)
        scale = 1.0 - progress * (1.0 - self._min_velocity_scale)
        self._last_velocity_scale = float(scale)
        return self._last_velocity_scale

    def publish_cmd_vel(self, vehicle_linear_velocity: float, yaw_rate: float) -> None:
        twist_msg = Twist()
        twist_msg.linear.x = float(vehicle_linear_velocity)
        twist_msg.angular.z = float(yaw_rate)
        self.twist_pub.publish(twist_msg)

    def broadcast_predicted_poses(
        self, x_points: np.ndarray, y_points: np.ndarray, yaws: np.ndarray
    ) -> None:
        now = self.get_clock().now().to_msg()
        for idx, (px, py, yaw) in enumerate(zip(x_points, y_points, yaws)):
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.base_footprint_frame_id
            transform.child_frame_id = f"predicted_ego_pose_{idx}"
            transform.transform.translation.x = float(px)
            transform.transform.translation.y = float(py)
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = float(np.sin(0.5 * yaw))
            transform.transform.rotation.w = float(np.cos(0.5 * yaw))
            self.tf_broadcaster.sendTransform(transform)

    def publish_cmd_vel_timer_callback(self) -> None:
        if not self.has_received_odometry:
            if not self.waiting_for_odometry_logged:
                self.get_logger().warning("Waiting for odometry. Publishing zero command.")
                self.waiting_for_odometry_logged = True
            self.publish_cmd_vel(0.0, 0.0)
            return

        self.waiting_for_odometry_logged = False

        try:
            with self._lock:
                objects = list(self._latest_objects)
                plan = self.planner.plan(self.lane_estimator, objects, self.prev_curvature)
                traffic_sign_scale = self._compute_traffic_sign_velocity_scale()

            self.prev_curvature = plan.curvature

            linear_velocity = calculate_decelerated_velocity(
                base_velocity=self.ego_target_velocity,
                eval_velocity=self.ego_target_velocity,
                curvature=plan.curvature,
                dt=self.deceleration_dt,
                deceleration_angle_maximum=self.deceleration_angle_maximum,
                deceleration_gain=self.deceleration_gain,
            )
            yaw_rate = linear_velocity * plan.curvature * self.yaw_rate_gain

            linear_velocity *= traffic_sign_scale
            yaw_rate *= traffic_sign_scale

            self.publish_cmd_vel(linear_velocity, yaw_rate)
            self.broadcast_predicted_poses(plan.x, plan.y, plan.yaw)

            # Debug log (~1 Hz): command vs. actual yaw rate
            self._log_counter += 1
            if self._log_counter * self.control_period >= 1.0:
                self._log_counter = 0
                self.get_logger().info(
                    f"kappa1={plan.curvature:+.3f} cmd_yaw_rate={yaw_rate:+.3f} "
                    f"odom_yaw_rate={self._odom_yaw_rate:+.3f} v={linear_velocity:.2f} "
                    f"lane={plan.has_valid_lane}"
                )

        except Exception as error:
            self.get_logger().error(f"Sampling MPC control cycle failed: {error}")
            self.publish_cmd_vel(0.0, 0.0)

    def stop_vehicle(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExtremumSeekingMpc()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Publishing stop command.")
    finally:
        node.stop_vehicle()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()