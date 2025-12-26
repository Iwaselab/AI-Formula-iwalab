#!/usr/bin/env python
# ==============================================================================
# インポート: 標準ライブラリ
# ==============================================================================
import math
import signal
import sys
from typing import List, Deque
from collections import deque
import numpy as np

# ==============================================================================
# インポート: ROS2関連
# ==============================================================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from aiformula_interfaces.msg import ObjectInfoMultiArray

try:
    from sensor_msgs_py import point_cloud2
except Exception:
    # ROS2 installations sometimes provide sensor_msgs.point_cloud2
    try:
        from sensor_msgs import point_cloud2
    except Exception:
        point_cloud2 = None


class LaneTracer(Node):
    def __init__(self):
        super().__init__('lane_tracer')

        # ======================================================================
        # パラメータ宣言: トピック名
        # ======================================================================
        self.declare_parameter('lane_topic', '/aiformula_perception/lane_line_publisher/lane_lines/center')
        self.declare_parameter('lane_topic_left', '/aiformula_perception/lane_line_publisher/lane_lines/left')
        self.declare_parameter('lane_topic_right', '/aiformula_perception/lane_line_publisher/lane_lines/right')
        # Default cmd_vel_topic changed to match vehicle plugin subscription
        self.declare_parameter('cmd_vel_topic', '/aiformula_control/extremum_seeking_mpc/cmd_vel')
        
        # ======================================================================
        # パラメータ宣言: Pure-Pursuit制御パラメータ
        # ======================================================================
        self.declare_parameter('lookahead', 0.8)
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('steer_gain', 1.0)
        # curvature-to-speed scaling gain for Pure-Pursuit
        self.declare_parameter('k_gain', 3.0)
        self.declare_parameter('max_angular', 2.0)
        
        # ======================================================================
        # パラメータ宣言: 履歴保存と補間軌道関連
        # ======================================================================
        # History size for lookahead points
        self.declare_parameter('history_size', 50)
        # Minimum number of history points needed for interpolation
        self.declare_parameter('min_history_for_interpolation', 5)
        # Distance threshold for detecting sudden jumps in lookahead point
        self.declare_parameter('jump_distance_threshold', 1.5)
        
        # ======================================================================
        # パラメータ宣言: 障害物回遼関連
        # ======================================================================
        # Obstacle avoidance parameters
        self.declare_parameter('obstacle_avoidance_enabled', True)
        self.declare_parameter('obstacle_avoidance_threshold', 1.5)  # th: threshold distance
        self.declare_parameter('obstacle_topic', '/aiformula_perception/object_publisher/pub_object')
        
        # Test obstacle parameters
        self.declare_parameter('test_obstacle_enabled', False)
        self.declare_parameter('test_obstacle_x', 5.0)  # x position in vehicle frame [m]
        self.declare_parameter('test_obstacle_y', 0.0)  # y position in vehicle frame [m]

        # ======================================================================
        # パラメータ取得
        # ======================================================================
        lane_topic = self.get_parameter('lane_topic').get_parameter_value().string_value
        lane_topic_left = self.get_parameter('lane_topic_left').get_parameter_value().string_value
        lane_topic_right = self.get_parameter('lane_topic_right').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.lookahead = float(self.get_parameter('lookahead').get_parameter_value().double_value)
        self.base_speed = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        self.steer_gain = float(self.get_parameter('steer_gain').get_parameter_value().double_value)
        self.k_gain = float(self.get_parameter('k_gain').get_parameter_value().double_value)
        self.max_angular = float(self.get_parameter('max_angular').get_parameter_value().double_value)
        self.history_size = int(self.get_parameter('history_size').get_parameter_value().integer_value)
        self.min_history_for_interpolation = int(self.get_parameter('min_history_for_interpolation').get_parameter_value().integer_value)
        self.jump_distance_threshold = float(self.get_parameter('jump_distance_threshold').get_parameter_value().double_value)
        self.obstacle_avoidance_enabled = bool(self.get_parameter('obstacle_avoidance_enabled').get_parameter_value().bool_value)
        self.obstacle_avoidance_threshold = float(self.get_parameter('obstacle_avoidance_threshold').get_parameter_value().double_value)
        obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value
        self.test_obstacle_enabled = bool(self.get_parameter('test_obstacle_enabled').get_parameter_value().bool_value)
        self.test_obstacle_x = float(self.get_parameter('test_obstacle_x').get_parameter_value().double_value)
        self.test_obstacle_y = float(self.get_parameter('test_obstacle_y').get_parameter_value().double_value)

        # ======================================================================
        # 内部変数の初期化
        # ======================================================================
        # 履歴保存用: 前方注視点の履歴をdequeで保存
        self.lookahead_history: Deque[tuple] = deque(maxlen=self.history_size)
        # 中心レーンが取得できない連続フレーム数のカウンター
        self.frames_without_center = 0
        # ジャンプ検出用: 前回の前方注視点
        self.prev_target = None
        # 障害物情報保存用: 車体座標系での(x, y)リスト
        self.obstacles = []  # List of (x, y) positions in vehicle frame

        if point_cloud2 is None:
            self.get_logger().error('point_cloud2 helper not found. Install sensor_msgs_py or ensure import works.')

        # ======================================================================
        # PublisherとSubscriberの設定
        # ======================================================================
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Subscriptions for center, left, and right lanes
        self.sub = self.create_subscription(
            PointCloud2,
            lane_topic,
            self.lane_callback,
            10)
        self.sub_left = self.create_subscription(
            PointCloud2,
            lane_topic_left,
            self.lane_callback_left,
            10)
        self.sub_right = self.create_subscription(
            PointCloud2,
            lane_topic_right,
            self.lane_callback_right,
            10)
        # 障害物情報のサブスクライプション（車体座標系に変換済み）
        self.sub_obstacles = self.create_subscription(
            ObjectInfoMultiArray,
            obstacle_topic,
            self.obstacle_callback,
            10)

        # Storage for fallback lanes
        self.last_left_points = []
        self.last_right_points = []
        self.last_center_points = []  # Store last valid center points for fallback
        
        # Log test obstacle info
        if self.test_obstacle_enabled:
            self.get_logger().info(
                f'Test obstacle enabled at ({self.test_obstacle_x:.2f}, {self.test_obstacle_y:.2f})'
            )
        
        self.get_logger().info(f'LaneTracer listening to [{lane_topic}] publishing to [{cmd_vel_topic}]')

    def __del__(self):
        """Send stop command when node is destroyed."""
        self.send_stop_command()

    def send_stop_command(self):
        """Publish zero velocity command to stop the vehicle."""
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            self.get_logger().info('Stop command sent')
        except Exception:
            pass  # Node may already be destroyed

    # ==========================================================================
    # 障害物情報の処理: すでに車体座標系に変換済みのデータを使用
    # ==========================================================================
    def obstacle_callback(self, msg: ObjectInfoMultiArray):
        """障害物情報を処理（object_publisherで既に車体座標系に変換済み）。
        
        ObjectInfoMultiArrayには以下の情報が含まれる:
        - x, y: 車体座標系での障害物位置 [m]
        - width: 障害物の幅 [m]
        - id: トラッキングID
        - confidence: 検出信頼度
        """
        self.obstacles = []
        
        for obj in msg.objects:
            obstacle_x = obj.x
            obstacle_y = obj.y
            
            # 前方かつ合理的な範囲内の障害物のみ保存
            if obstacle_x > 0.1 and obstacle_x < 15.0 and abs(obstacle_y) < 5.0:
                self.obstacles.append((obstacle_x, obstacle_y))
                self.get_logger().debug(
                    f'Obstacle detected: vehicle({obstacle_x:.2f}m, {obstacle_y:.2f}m), '
                    f'width: {obj.width:.2f}m, confidence: {obj.confidence:.2f}'
                )

    def lane_callback(self, msg: PointCloud2):
        # extract (x,y) points from pointcloud
        points = []  # type: List[tuple]
        if point_cloud2 is None:
            return

        try:
            for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
                x, y = p[0], p[1]
                # we only consider points in front of vehicle
                if x >= 0.0:
                    points.append((float(x), float(y)))
        except Exception as e:
            self.get_logger().warn(f'Failed to read points: {e}')
            return

        if not points:
            # Center lane unavailable -> increment counter and try fallback
            self.frames_without_center += 1
            self.process_with_fallback()
            return

        # Reset counter when center lane is available
        self.frames_without_center = 0
        # Store valid center points for fallback
        self.last_center_points = points
        self.process_lane_points(points, store_history=True)

    def process_lane_points(self, points: List[tuple], store_history: bool = False):
        """Process lane points and publish control commands."""
        # find target point at lookahead distance
        target = self.find_target_point(points, self.lookahead)

        # Get obstacles including test obstacle
        obstacles_to_avoid = self.get_obstacles_for_avoidance()
        
        # Apply obstacle avoidance if enabled
        if self.obstacle_avoidance_enabled and obstacles_to_avoid:
            avoided_target = self.avoid_obstacles(target, obstacles_to_avoid)
            if avoided_target is not None:
                target = avoided_target

        # Check if target point jumped significantly from previous one
        if store_history and self.prev_target is not None and target != (0.0, 0.0):
            distance = math.hypot(target[0] - self.prev_target[0], target[1] - self.prev_target[1])
            if distance > self.jump_distance_threshold:
                # Target jumped too much - use interpolation if available
                if len(self.lookahead_history) >= self.min_history_for_interpolation:
                    interpolated_target = self.interpolate_from_history()
                    if interpolated_target is not None:
                        self.get_logger().info(
                            f'Detected jump in lookahead point (distance: {distance:.3f}m), '
                            f'using interpolated trajectory'
                        )
                        # Use interpolated target instead
                        target = interpolated_target

        # Store target point in history if requested
        if store_history and target != (0.0, 0.0):
            self.lookahead_history.append(target)
            self.prev_target = target

        # angle (alpha) to target in vehicle frame
        alpha = math.atan2(target[1], target[0])

        # Pure-Pursuit curvature calculation
        # k = 2 * sin(alpha) / Ld
        if self.lookahead <= 0.0:
            curvature = 0.0
        else:
            curvature = 2.0 * math.sin(alpha) / float(self.lookahead)

        # Angular velocity from curvature: omega = v * k
        angular_z = float(self.base_speed * curvature)
        # apply steering gain and clamp
        angular_z = max(-self.max_angular, min(self.max_angular, self.steer_gain * angular_z))

        # speed scaling based on curvature magnitude (reduce speed for sharp turns)
        speed = float(self.base_speed / (1.0 + self.k_gain * abs(curvature)))
        # enforce a conservative minimum speed
        if speed < 0.05:
            speed = 0.0

        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = float(angular_z)
        self.pub.publish(twist)

    @staticmethod
    def find_target_point(points: List[tuple], lookahead: float) -> tuple:
        # points: list of (x,y) in vehicle frame. Choose the first point with distance >= lookahead.
        best = None
        for x, y in points:
            d = math.hypot(x, y)
            if d >= lookahead:
                return (x, y)
            if best is None or d > math.hypot(best[0], best[1]):
                best = (x, y)
        return best if best is not None else (0.0, 0.0)

    def get_obstacles_for_avoidance(self) -> List[tuple]:
        """Get list of obstacles for avoidance (including test obstacle).
        
        Returns:
            List of (x, y) obstacle positions in vehicle frame
        """
        obstacles = list(self.obstacles)  # Copy actual detected obstacles
        
        # Add test obstacle if enabled
        if self.test_obstacle_enabled:
            obstacles.append((self.test_obstacle_x, self.test_obstacle_y))
            self.get_logger().debug(
                f'Using test obstacle at ({self.test_obstacle_x:.2f}, {self.test_obstacle_y:.2f})',
                throttle_duration_sec=1.0
            )
        
        return obstacles
    
    # ==========================================================================
    # 障害物回遼アルゴリズム: 2つの円の交点を利用
    # ==========================================================================
    def avoid_obstacles(self, original_target: tuple, obstacles: List[tuple] = None) -> tuple:
        """障害物を回遼するために、障害物周囲の円上に前方注視点を移動。
        
        アルゴリズム:
        - 円Cr: ロボット位置(0,0)を中心、半径ld（前方注視距離）
        - 円Co: 障害物位置(xo,yo)を中心、半径th（回遼閾値）
        - この2つの円の交点上に前方注視点を移動
        - 前回の注視点に近い方の交点を選択
        
        Returns:
            障害物回遼が必要な場合は修正された注視点、そうでなければNone
        """
        # Use provided obstacles or fall back to self.obstacles
        if obstacles is None:
            obstacles = self.obstacles
        
        if not obstacles or original_target == (0.0, 0.0):
            return None
        
        # Robot position is at origin (0, 0) in vehicle frame
        xr, yr = 0.0, 0.0
        
        # Check each obstacle
        for xo, yo in obstacles:
            # Distance from robot to obstacle
            dist_to_obstacle = math.hypot(xo - xr, yo - yr)
            
            # Check if obstacle is within lookahead range
            # For circle intersection to work: |r1 - r2| < d < r1 + r2
            # where r1=lookahead, r2=threshold
            min_dist = abs(self.lookahead - self.obstacle_avoidance_threshold)
            max_dist = self.lookahead + self.obstacle_avoidance_threshold
            
            if min_dist < dist_to_obstacle < max_dist:
                self.get_logger().info(
                    f'Obstacle detected at ({xo:.2f}, {yo:.2f}), distance: {dist_to_obstacle:.2f}m'
                )
                
                # Calculate intersection of two circles:
                # Circle Cr: center at (xr, yr), radius ld (lookahead distance)
                # Circle Co: center at (xo, yo), radius th (threshold)
                intersections = self.circle_intersection(
                    xr, yr, self.lookahead,
                    xo, yo, self.obstacle_avoidance_threshold
                )
                
                if intersections:
                    # Choose intersection point closer to previous target
                    if self.prev_target is not None:
                        new_target = self.choose_closer_point(intersections, self.prev_target)
                    else:
                        # If no previous target, choose point further from obstacle
                        new_target = self.choose_point_away_from_obstacle(intersections, xo, yo)
                    
                    self.get_logger().info(
                        f'Avoidance: moving target from ({original_target[0]:.2f}, {original_target[1]:.2f}) '
                        f'to ({new_target[0]:.2f}, {new_target[1]:.2f})'
                    )
                    return new_target
        
        return None

    @staticmethod
    def circle_intersection(x1: float, y1: float, r1: float, 
                           x2: float, y2: float, r2: float) -> List[tuple]:
        """Calculate intersection points of two circles.
        
        Returns list of (x, y) tuples for intersection points (0, 1, or 2 points).
        """
        # Distance between centers
        d = math.hypot(x2 - x1, y2 - y1)
        
        # Check if circles intersect
        if d > r1 + r2:  # Circles too far apart
            return []
        if d < abs(r1 - r2):  # One circle inside the other
            return []
        if d == 0 and r1 == r2:  # Circles are identical
            return []
        
        # Calculate intersection points
        a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
        h = math.sqrt(r1 * r1 - a * a)
        
        # Point on line between centers
        x3 = x1 + a * (x2 - x1) / d
        y3 = y1 + a * (y2 - y1) / d
        
        # Two intersection points
        intersect1 = (
            x3 + h * (y2 - y1) / d,
            y3 - h * (x2 - x1) / d
        )
        intersect2 = (
            x3 - h * (y2 - y1) / d,
            y3 + h * (x2 - x1) / d
        )
        
        return [intersect1, intersect2]

    @staticmethod
    def choose_closer_point(points: List[tuple], reference: tuple) -> tuple:
        """Choose point closer to reference point."""
        if len(points) == 1:
            return points[0]
        
        dist1 = math.hypot(points[0][0] - reference[0], points[0][1] - reference[1])
        dist2 = math.hypot(points[1][0] - reference[0], points[1][1] - reference[1])
        
        return points[0] if dist1 <= dist2 else points[1]

    @staticmethod
    def choose_point_away_from_obstacle(points: List[tuple], xo: float, yo: float) -> tuple:
        """Choose point further away from obstacle."""
        if len(points) == 1:
            return points[0]
        
        dist1 = math.hypot(points[0][0] - xo, points[0][1] - yo)
        dist2 = math.hypot(points[1][0] - xo, points[1][1] - yo)
        
        return points[0] if dist1 >= dist2 else points[1]

    def lane_callback_left(self, msg: PointCloud2):
        """Store left lane points for fallback."""
        points = []
        if point_cloud2 is None:
            return
        try:
            for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
                x, y = p[0], p[1]
                if x >= 0.0:
                    points.append((float(x), float(y)))
        except Exception as e:
            self.get_logger().warn(f'Failed to read left lane points: {e}')
            return
        self.last_left_points = points

    def lane_callback_right(self, msg: PointCloud2):
        """Store right lane points for fallback."""
        points = []
        if point_cloud2 is None:
            return
        try:
            for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
                x, y = p[0], p[1]
                if x >= 0.0:
                    points.append((float(x), float(y)))
        except Exception as e:
            self.get_logger().warn(f'Failed to read right lane points: {e}')
            return
        self.last_right_points = points

    def process_with_fallback(self):
        """Fallback logic: interpolated trajectory > last center > left > right > stop."""
        # Try to use interpolated trajectory from history
        if len(self.lookahead_history) >= self.min_history_for_interpolation:
            interpolated_target = self.interpolate_from_history()
            if interpolated_target is not None:
                self.get_logger().info(f'Using interpolated trajectory (history: {len(self.lookahead_history)})')
                # Create control command from interpolated target
                self.process_interpolated_target(interpolated_target)
                return
        
        if self.last_center_points:
            # Use last valid center points
            self.process_lane_points(self.last_center_points)
        elif self.last_left_points:
            # Prefer left lane
            self.process_lane_points(self.last_left_points)
        elif self.last_right_points:
            # Fall back to right lane
            self.process_lane_points(self.last_right_points)
        else:
            # No fallback available: stop
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

    def interpolate_from_history(self) -> tuple:
        """Generate interpolated target point from lookahead history."""
        if len(self.lookahead_history) < self.min_history_for_interpolation:
            return None
        
        try:
            # Convert history to numpy array for easier processing
            history_array = np.array(list(self.lookahead_history))
            x_values = history_array[:, 0]
            y_values = history_array[:, 1]
            
            # Use polynomial fitting to extrapolate the trajectory
            # Fit a 2nd degree polynomial to the historical points
            degree = min(2, len(self.lookahead_history) - 1)
            
            # Create time indices for the historical points
            t = np.arange(len(self.lookahead_history))
            
            # Fit polynomials for x and y separately
            poly_x = np.polyfit(t, x_values, degree)
            poly_y = np.polyfit(t, y_values, degree)
            
            # Extrapolate to the next point
            # Use a weighted approach that considers the velocity trend
            next_t = len(self.lookahead_history)
            predicted_x = np.polyval(poly_x, next_t)
            predicted_y = np.polyval(poly_y, next_t)
            
            # Ensure the predicted point is reasonable (not too far from last point)
            last_point = self.lookahead_history[-1]
            distance = math.hypot(predicted_x - last_point[0], predicted_y - last_point[1])
            
            # If the predicted point is too far, use a simpler velocity-based prediction
            if distance > self.lookahead * 2.0:
                # Use average velocity from last few points
                if len(self.lookahead_history) >= 3:
                    recent_points = list(self.lookahead_history)[-3:]
                    dx = recent_points[-1][0] - recent_points[0][0]
                    dy = recent_points[-1][1] - recent_points[0][1]
                    predicted_x = last_point[0] + dx / 2.0
                    predicted_y = last_point[1] + dy / 2.0
                else:
                    predicted_x = last_point[0]
                    predicted_y = last_point[1]
            
            return (float(predicted_x), float(predicted_y))
            
        except Exception as e:
            self.get_logger().warn(f'Failed to interpolate from history: {e}')
            return None

    def process_interpolated_target(self, target: tuple):
        """Process an interpolated target point and publish control commands."""
        # angle (alpha) to target in vehicle frame
        alpha = math.atan2(target[1], target[0])

        # Pure-Pursuit curvature calculation
        # k = 2 * sin(alpha) / Ld
        if self.lookahead <= 0.0:
            curvature = 0.0
        else:
            # Use actual distance to target for curvature calculation
            actual_distance = math.hypot(target[0], target[1])
            if actual_distance > 0.0:
                curvature = 2.0 * math.sin(alpha) / actual_distance
            else:
                curvature = 0.0

        # Angular velocity from curvature: omega = v * k
        # Reduce speed when using interpolated trajectory for safety
        interpolation_speed_factor = 0.9
        reduced_speed = self.base_speed * interpolation_speed_factor
        angular_z = float(reduced_speed * curvature)
        # apply steering gain and clamp
        angular_z = max(-self.max_angular, min(self.max_angular, self.steer_gain * angular_z))

        # speed scaling based on curvature magnitude
        speed = float(reduced_speed / (1.0 + self.k_gain * abs(curvature)))
        # enforce a conservative minimum speed
        if speed < 0.05:
            speed = 0.0

        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = float(angular_z)
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LaneTracer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        # Send stop command before destroying node
        node.get_logger().info('Shutdown signal received, stopping vehicle...')
        node.send_stop_command()
        try:
            node.destroy_node()
        except Exception:
            pass  # Node may already be destroyed
        try:
            rclpy.shutdown()
        except Exception:
            pass  # ROS may already be shut down


if __name__ == '__main__':
    main()
