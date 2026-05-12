#!/usr/bin/env python
# Pythonスクリプトとして実行することを指定

from typing import List
# List型ヒントを使用するためのimport

import numpy as np
# 数値計算ライブラリNumPyをimport

from enum import IntEnum
# 整数型Enumを使うためのimport

import rclpy
# ROS2 Pythonクライアントライブラリ

from rclpy.node import Node
# ROS2ノードクラスをimport

from geometry_msgs.msg import Twist
# 速度指令(Twist型)メッセージ

from sensor_msgs.msg import Imu
# IMUメッセージ型

from can_msgs.msg import Frame
# CANフレームメッセージ型

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# QoS設定用クラス群

from common_python.get_ros_parameter import get_ros_parameter
# ROSパラメータ取得用関数


# ===============================
# CAN通信用QoS設定
# ===============================

qos_can = QoSProfile(

    reliability=ReliabilityPolicy.BEST_EFFORT,
    # 通信品質をBEST_EFFORTに設定
    # （多少欠損しても高速性優先）

    history=HistoryPolicy.KEEP_LAST,
    # 最新データのみ保持する方式

    depth=10
    # 最大10個保持
)


# ===============================
# 左右車輪番号定義
# ===============================

class DriveWheel(IntEnum):

    LEFT = 0
    # 左車輪番号

    RIGHT = 1
    # 右車輪番号

    NUM_DRIVE_WHEELS = 2
    # 駆動輪数


# ===============================
# モータ制御ノード
# ===============================

class MotorController(Node):

    def __init__(self):

        super().__init__('motor_controller')
        # ノード名をmotor_controllerとして初期化

        self.get_ros_params()
        # ROSパラメータ読み込み

        # ===============================
        # Publisher & Subscriber
        # ===============================

        buffer_size = 10
        # 通常通信用バッファサイズ

        self.twist_sub = self.create_subscription(
            Twist,
            'sub_speed_command',
            self.twist_callback,
            buffer_size
        )
        # 速度指令Subscriber作成

        self.imu_sub = self.create_subscription(
            Imu,
            'sub_imu',
            self.imu_callback,
            buffer_size
        )
        # IMU Subscriber作成

        self.can_sub = self.create_subscription(
            Frame,
            'sub_can',
            self.can_callback,
            qos_can
        )
        # CAN Subscriber作成

        self.can_pub = self.create_publisher(
            Frame,
            'pub_can',
            buffer_size
        )
        # CAN Publisher作成

        self.publish_timer = self.create_timer(
            self.publish_timer_loop_duration,
            self.publish_canframe_callback
        )
        # 一定周期でCAN送信するタイマー作成

        self.frame_msg = Frame()
        # CAN送信用Frame生成

        self.frame_msg.header.frame_id = "can0"
        # 使用CANインタフェース名

        self.frame_msg.id = 0x210
        # CAN ID設定

        self.frame_msg.dlc = 8
        # データ長8byte

        self.yaw_timer = self.create_timer(
            self.pd_dt,
            self.control_loop_callback
        )
        # ヨーレートPID制御周期タイマー

        self.vel_timer = self.create_timer(
            self.pd_dt,
            self.control_velo_callback
        )
        # 速度PID制御周期タイマー

        # ===============================
        # ヨーレートPID用変数
        # ===============================

        self.omega_now = 0.0
        # 現在ヨーレート[rad/s]

        self.omega_ref = 0.0
        # 目標ヨーレート[rad/s]

        self.e_prev = 0.0
        # 前回誤差

        self.u_pd = 0.0
        # PID出力

        self.ie = 0.0
        # 積分値

        self.prev_omega_ref = 0.0
        # 前回目標ヨーレート

        self.last_can_msg = None
        # 最新CANメッセージ保存用

        # ===============================
        # 速度PID用変数
        # ===============================

        self.v_ref = 0.0
        # 目標速度[m/s]

        self.u_v = 0.0
        # 速度PID出力

        self.V_e_prev = 0.0
        # 前回速度誤差

        self.V_ie = 0.0
        # 積分項

    # ===============================
    # ROSパラメータ取得
    # ===============================

    def get_ros_params(self):

        self.diameter = get_ros_parameter(
            self,
            "wheel.diameter"
        )
        # 車輪直径[m]

        self.tread = get_ros_parameter(
            self,
            "wheel.tread"
        )
        # 車輪間距離[m]

        self.gear_ratio = get_ros_parameter(
            self,
            "wheel.gear_ratio"
        )
        # ギア比

        self.publish_timer_loop_duration = get_ros_parameter(
            self,
            "publish_timer_loop_duration"
        )
        # CAN送信周期[s]

        # ===============================
        # ヨーレートPIDゲイン
        # ===============================

        self.pd_dt = get_ros_parameter(self, "pd.dt")
        # PID制御周期

        self.kp = get_ros_parameter(self, "pd.kp")
        # Pゲイン

        self.ki = get_ros_parameter(self, "pd.ki")
        # Iゲイン

        self.kd = get_ros_parameter(self, "pd.kd")
        # Dゲイン

        self.max_output = get_ros_parameter(
            self,
            "pd.max_output"
        )
        # PID出力上限

        self.min_output = get_ros_parameter(
            self,
            "pd.min_output"
        )
        # PID出力下限

        # ===============================
        # 速度PIDゲイン
        # ===============================

        self.kp_v = get_ros_parameter(self, "pd.kp_v")
        # 速度Pゲイン

        self.ki_v = get_ros_parameter(self, "pd.ki_v")
        # 速度Iゲイン

        self.kd_v = get_ros_parameter(self, "pd.kd_v")
        # 速度Dゲイン

        self.max_output_v = get_ros_parameter(
            self,
            "pd.max_output_v"
        )
        # 速度PID出力上限

        self.min_output_v = get_ros_parameter(
            self,
            "pd.min_output_v"
        )
        # 速度PID出力下限

    # ===============================
    # Twist受信コールバック
    # ===============================

    def twist_callback(self, msg):

        self.omega_ref = msg.angular.z
        # 目標ヨーレート更新

        self.v_ref = msg.linear.x
        # 目標速度更新

        rpm = self.toRefRPM(
            msg.linear.x,
            msg.angular.z,
            self.u_pd,
            self.u_V
        )
        # 左右モータRPM計算

        cmd_left = self.toCanCmd(
            rpm[DriveWheel.LEFT]
        )
        # 左モータRPM→CANデータ変換

        cmd_right = self.toCanCmd(
            rpm[DriveWheel.RIGHT]
        )
        # 右モータRPM→CANデータ変換

        can_data = cmd_right + cmd_left
        # CANデータ結合

        self.frame_msg.data = can_data
        # CAN送信データへ格納

    # ===============================
    # IMU受信コールバック
    # ===============================

    def imu_callback(self, msg: Imu):

        self.omega_now = msg.angular_velocity.z
        # IMUからヨーレート取得

    # ===============================
    # CAN受信コールバック
    # ===============================

    def can_callback(self, msg: Frame):

        RPM_ID = 0x711
        # モータRPM受信用CAN ID

        if msg.id != RPM_ID:
            return
        # 指定ID以外は無視

        if self.last_can_msg is None:

            self.get_logger().info(
                "Subscribe CAN RPM Frame !"
            )
            # 初回受信ログ表示

        self.last_can_msg = msg
        # 最新CANメッセージ保存

    # ===============================
    # CAN送信タイマー
    # ===============================

    def publish_canframe_callback(self):

        self.can_pub.publish(self.frame_msg)
        # CANフレーム送信

    # ===============================
    # ヨーレートPID制御
    # ===============================

    def control_loop_callback(self):

        omega_ref = self.omega_ref
        # 現在目標ヨーレート取得

        e = omega_ref - self.omega_now
        # 誤差計算

        de = (e - self.e_prev) / self.pd_dt
        # 微分項計算

        u = (
            self.kp * e
            + self.kd * de
            + self.ki * self.ie
        )
        # PID出力計算

        if u > self.max_output:

            u = self.max_output
            # 上限飽和

        elif u < self.min_output:

            u = self.min_output
            # 下限飽和

        else:

            self.ie += e * self.pd_dt
            # 積分項更新

        self.u_pd = u
        # PID出力保存

        self.e_prev = e
        # 前回誤差更新

        self.prev_omega_ref = omega_ref
        # 前回目標値更新

    # ===============================
    # 速度PID制御
    # ===============================

    def control_velo_callback(self):

        V_now = self.calc_linear_velocity_from_can()
        # 現在速度算出

        V_ref = self.v_ref
        # 目標速度取得

        V_e = V_ref - V_now
        # 速度誤差

        V_de = (
            V_e - self.V_e_prev
        ) / self.pd_dt
        # 微分項

        u_V = (
            self.kp_v * V_e
            + self.kd_v * V_de
            + self.ki_v * self.V_ie
        )
        # PID出力

        if u_V > self.max_output_v:

            u_V = self.max_output_v
            # 上限飽和

        elif u_V < self.min_output_v:

            u_V = self.min_output_v
            # 下限飽和

        else:

            self.V_ie += V_e * self.pd_dt
            # 積分更新

        self.u_V = u_V
        # 出力保存

        self.V_e_prev = V_e
        # 前回誤差更新

    # ===============================
    # 速度→左右RPM変換
    # ===============================

    def toRefRPM(
        self,
        linear_velocity,
        angular_velocity,
        u_pd,
        u_V
    ):

        wheel_angular_velocities = np.zeros(
            DriveWheel.NUM_DRIVE_WHEELS
        )
        # 左右輪角速度配列生成

        wheel_angular_velocities[
            DriveWheel.LEFT
        ] = (
            (linear_velocity / (self.diameter * 0.5))
            + u_V
            - (self.tread / self.diameter)
            * angular_velocity
            - u_pd
        )
        # 左輪角速度計算

        wheel_angular_velocities[
            DriveWheel.RIGHT
        ] = (
            (linear_velocity / (self.diameter * 0.5))
            + u_V
            + (self.tread / self.diameter)
            * angular_velocity
            + u_pd
        )
        # 右輪角速度計算

        minute_to_second = 60.
        # 秒→分変換係数

        rpm = (
            wheel_angular_velocities
            * (minute_to_second / (2. * np.pi))
        )
        # rad/s→rpm変換

        return (
            rpm * self.gear_ratio
        ).tolist()
        # ギア比考慮して返却

    # ===============================
    # RPM→CANデータ変換
    # ===============================

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:

        rounded = round(rpm)
        # RPM整数化

        bytes = rounded.to_bytes(
            4,
            "little",
            signed=True
        )
        # 4byte little endian変換

        return list(bytes)
        # list型へ変換

    # ===============================
    # CANから現在速度計算
    # ===============================

    def calc_linear_velocity_from_can(self):

        if self.last_can_msg is None:

            return 0.0
            # CAN未受信なら0返却

        data = self.last_can_msg.data
        # CANデータ取得

        rpm_right = int.from_bytes(
            data[0:4],
            byteorder='little',
            signed=True
        )
        # 右モータRPM復元

        rpm_left = int.from_bytes(
            data[4:8],
            byteorder='little',
            signed=True
        )
        # 左モータRPM復元

        rpm_avg = 0.5 * (
            rpm_right + rpm_left
        )
        # 平均RPM

        wheel_radius = self.diameter * 0.5
        # 車輪半径[m]

        v = (
            (rpm_avg * 2.0 * np.pi / 60.0)
            * wheel_radius
            / self.gear_ratio
        )
        # rpm→m/s変換

        return v
        # 現在速度返却


# ===============================
# メイン関数
# ===============================

def main(args=None):

    rclpy.init(args=args)
    # ROS2初期化

    motor_controller = MotorController()
    # ノード生成

    rclpy.spin(motor_controller)
    # ノード実行

    motor_controller.destroy_node()
    # ノード削除

    rclpy.shutdown()
    # ROS2終了


# ===============================
# Python実行時エントリ
# ===============================

if __name__ == '__main__':

    main()
    # main関数実行