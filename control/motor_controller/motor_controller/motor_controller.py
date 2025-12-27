#!/usr/bin/env python
from typing import List
import numpy as np
from enum import IntEnum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame

from common_python.get_ros_parameter import get_ros_parameter


class DriveWheel(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_DRIVE_WHEELS = 2


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.get_ros_params()

        # Publisher & Subscriber
        buffer_size = 10
        self.twist_sub = self.create_subscription(Twist, 'sub_speed_command', self.twist_callback, buffer_size)
        self.imu_sub = self.create_subscription(Imu, 'sub_imu', self.imu_callback, buffer_size)  # IMU subscriber 菅澤
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.publish_timer = self.create_timer(self.publish_timer_loop_duration, self.publish_canframe_callback)
        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = "can0"        # Default can0
        self.frame_msg.id = 0x210                      # MotorController CAN ID : 0x210
        self.frame_msg.dlc = 8                         # Data length
        self.control_timer = self.create_timer(self.pd_dt, self.control_loop_callback)

        # PD制御用変数初期化 (菅澤)
        self.omega_now = 0.0     # IMUから取得する現在のヨーレート [rad/s]
        self.omega_ref = 0.0     # 目標ヨーレート [rad/s]
        self.e_prev = 0.0        # 1周期前の差
        self.u_pd = 0.0          # 出力 [rad/s]

    def get_ros_params(self):
        self.diameter = get_ros_parameter(self, "wheel.diameter")
        self.tread = get_ros_parameter(self, "wheel.tread")
        self.gear_ratio = get_ros_parameter(self, "wheel.gear_ratio")
        self.publish_timer_loop_duration = get_ros_parameter(self, "publish_timer_loop_duration")

        # --- PD params (菅澤)---
        self.pd_dt = get_ros_parameter(self, "pd.dt")
        self.kp = get_ros_parameter(self, "pd.kp")
        self.kd = get_ros_parameter(self, "pd.kd")
        self.max_output = get_ros_parameter(self, "pd.max_output")
        self.min_output = get_ros_parameter(self, "pd.min_output")

    def twist_callback(self, msg):
        self.omega_ref = msg.angular.z  # 目標ヨーレート更新(菅澤)
        rpm = self.toRefRPM(msg.linear.x, msg.angular.z, self.u_pd)
        cmd_left = self.toCanCmd(rpm[DriveWheel.LEFT])
        cmd_right = self.toCanCmd(rpm[DriveWheel.RIGHT])        
        can_data = cmd_right + cmd_left
        self.frame_msg.data = can_data
    
    def imu_callback(self, msg: Imu):
    # まずはヨーレートだけ取得（PDで使う予定）
        self.omega_now = msg.angular_velocity.z

    #取得した場合に出力(テスト用)
        self.get_logger().debug(
        f"IMU yaw rate received: {self.omega_now:.4f} rad/s"
    )

    def publish_canframe_callback(self):
        self.can_pub.publish(self.frame_msg)

    def control_loop_callback(self):
        # 誤差計算
        e = self.omega_ref - self.omega_now

        # 微分項（固定周期 dt）
        de = (e - self.e_prev) / self.pd_dt

        # PD制御
        u = self.kp * e + self.kd * de

        # --- 飽和計算 ---
        if u > self.max_output:
            u = self.max_output
        elif u < self.min_output:
            u = self.min_output

        # 永続更新
        self.u_pd = u
        self.e_prev = e

#  Velocity -> RPM Calc
#  V_right = (V + tread/2 * w)   [m/s]
#  V_left = (V - tread / 2 * w ) [m/s]
#  w_right = V/r + (d/2r) * w    [rad/s]
#  w_left = V/r - (d/2r) * w     [rad/s]
#  rpm = w * 60 / 2* pi          [rpm]
#  rpm = rpm * gear_ratio        [rpm]

    def toRefRPM(self, linear_velocity, angular_velocity, u_pd):  # Calc Motor ref rad/s
        wheel_angular_velocities = np.zeros(DriveWheel.NUM_DRIVE_WHEELS)
        wheel_angular_velocities[DriveWheel.LEFT] = (
            linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity # [rad/s]
        wheel_angular_velocities[DriveWheel.RIGHT] = (
            linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity # [rad/s]
        
        #前進時のみPD制御の補正を左右に加える (菅澤)
        if linear_velocity >= 0.0:
            wheel_angular_velocities[DriveWheel.LEFT]  -= u_pd
            wheel_angular_velocities[DriveWheel.RIGHT] += u_pd
        
        #値が処理の途中で変化しないようにに変数に代入してから使う
        left_w  = wheel_angular_velocities[DriveWheel.LEFT]
        right_w = wheel_angular_velocities[DriveWheel.RIGHT]
        
        #左右の回転方向が逆になる場合は負の指令値を8.00rad/sに固定
        if left_w * right_w < 0.0:
            if left_w < 0.0:
                wheel_angular_velocities[DriveWheel.LEFT] = 8.00
            if right_w < 0.0:
                wheel_angular_velocities[DriveWheel.RIGHT] = 8.00
        
        #以下は変更なし
        minute_to_second = 60.
        rpm = wheel_angular_velocities * (minute_to_second / (2. * np.pi))
        if rpm[DriveWheel.LEFT] * rpm[DriveWheel.RIGHT] < 0.0:
            rpm[:] = 0.0
            self.get_logger().debug(f"Preventing in-situ rotation ! (rpm: {rpm})")
        return (rpm * self.gear_ratio).tolist()

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
