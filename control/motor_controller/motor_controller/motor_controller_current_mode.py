#!/usr/bin/env python3
"""
motor_controller_current_mode.py

既存の motor_controller ノードの外部インターフェースを維持したまま、
モータドライバを「電流制御モード」で使用するための実装。

外部インターフェース
--------------------
入力:
    sub_speed_command : geometry_msgs/msg/Twist
        linear.x  = 車体並進速度目標値 v_ref [m/s]
        angular.z = 車体ヨー角速度目標値 omega_ref [rad/s]

    sub_imu : sensor_msgs/msg/Imu
        angular_velocity.z = 実測ヨー角速度 omega [rad/s]

    sub_can : can_msgs/msg/Frame
        CAN ID 0x711 = 左右モータ回転数
        CAN ID 0x712 = 左右モータ電流のフィードバック値

出力:
    pub_can : can_msgs/msg/Frame
        CAN ID 0x210
        data[0:4] = 右モータの指令値
        data[4:8] = 左モータの指令値

重要な設計方針
--------------
1. Twist の意味は従来どおり「速度・ヨー角速度目標値」とする。
   新しいトピックや新しいメッセージ型は導入しない。

2. 速度偏差から左右共通の電流成分 i_v を生成する。
   ヨー角速度偏差から左右差動の電流成分 i_omega を生成する。

       i_right = i_v + i_omega
       i_left  = i_v - i_omega

3. モータドライバの電流指令換算は、確認済みの関係

       I_cmd [A] = raw_command / 1000 * 50

   を使用する。したがって、

       raw_command = 20 * I_cmd [A]

   である。

4. pd.max_output_v, pd.min_output_v は、並進速度制御器が出力する
   左右共通電流成分の上限・下限 [A] として使用する。

   pd.max_output, pd.min_output は、ヨー角速度制御器が出力する
   左右差動電流成分の上限・下限 [A] として使用する。

   従来の速度制御モード用ゲインをそのまま流用せず、
   電流 [A] を出力する制御器として再調整する必要がある。
"""

from enum import IntEnum
from typing import List, Tuple

import numpy as np
import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from common_python.get_ros_parameter import get_ros_parameter


# ===============================
# CAN通信用QoS設定
# ===============================

qos_can = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ===============================
# 左右車輪番号定義
# ===============================

class DriveWheel(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_DRIVE_WHEELS = 2


# ===============================
# モータ制御ノード
# ===============================

class MotorController(Node):

    # CAN ID
    CAN_COMMAND_ID = 0x210
    CAN_RPM_FEEDBACK_ID = 0x711
    CAN_CURRENT_FEEDBACK_ID = 0x712

    # モータドライバの電流指令換算
    #
    # raw = 1000 のとき 50 A
    # I_cmd [A] = raw / 1000 * 50
    # raw = I_cmd [A] * 20
    DRIVER_CURRENT_LIMIT_AMP = 50.0
    DRIVER_COMMAND_FULL_SCALE = 1000
    AMP_TO_COMMAND_GAIN = (
        DRIVER_COMMAND_FULL_SCALE / DRIVER_CURRENT_LIMIT_AMP
    )

    def __init__(self):
        super().__init__('motor_controller')

        self.get_ros_params()

        # ===============================
        # 制御状態の初期化
        # ===============================

        # 目標値
        self.v_ref = 0.0
        self.omega_ref = 0.0

        # 実測値
        self.omega_now = 0.0
        self.last_can_msg = None

        # ヨー角速度PID用状態
        self.e_prev = 0.0
        self.ie = 0.0
        self.u_pd = 0.0

        # 並進速度PID用状態
        self.V_e_prev = 0.0
        self.V_ie = 0.0
        self.u_V = 0.0

        # 現在の左右電流指令 [A]
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0

        # デバッグ用変数
        self.rpm_right_debug = 0.0
        self.rpm_left_debug = 0.0
        self.rpm_avg_debug = 0.0
        self.v_now_debug = 0.0
        self.V_e_debug = 0.0

        # CURRENT_ID = 0x712 から受信した値。
        # ドライバ側のTPDO4がA単位を返している場合は[A]である。
        # スケーリングされた整数値の場合は、can_callback()内で換算する。
        self.current_right_feedback_debug = 0.0
        self.current_left_feedback_debug = 0.0
        self.torque_right_debug = 0.0
        # 右モータトルク[Nm×100]
        self.torque_left_debug = 0.0
        # 左モータトルク[Nm×100]

        # ===============================
        # Publisher & Subscriber
        # ===============================

        buffer_size = 10

        self.twist_sub = self.create_subscription(
            Twist,
            'sub_speed_command',
            self.twist_callback,
            buffer_size,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'sub_imu',
            self.imu_callback,
            buffer_size,
        )

        self.can_sub = self.create_subscription(
            Frame,
            'sub_can',
            self.can_callback,
            qos_can,
        )

        self.can_pub = self.create_publisher(
            Frame,
            'pub_can',
            buffer_size,
        )

        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            'debug/controller_state',
            18,
        )

        # ===============================
        # CAN送信フレーム
        # ===============================

        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = "can0"
        self.frame_msg.id = self.CAN_COMMAND_ID
        self.frame_msg.dlc = 8

        # 起動直後に未初期化データを送らないよう、ゼロ指令で初期化する。
        self.frame_msg.data = [0] * 8

        # ===============================
        # タイマー
        # ===============================

        # 左右電流指令をCAN送信する周期
        self.publish_timer = self.create_timer(
            self.publish_timer_loop_duration,
            self.publish_canframe_callback,
        )

        # ヨー角速度制御周期
        self.yaw_timer = self.create_timer(
            self.pd_dt,
            self.control_yaw_rate_callback,
        )

        # 並進速度制御周期
        self.vel_timer = self.create_timer(
            self.pd_dt,
            self.control_velocity_callback,
        )

        self.get_logger().info(
            "MotorController started in current-command mode. "
            "Twist remains (v_ref, omega_ref)."
        )

    # ===============================
    # ROSパラメータ取得
    # ===============================

    def get_ros_params(self) -> None:

        self.diameter = get_ros_parameter(
            self,
            "wheel.diameter",
        )
        # 車輪直径 [m]

        self.tread = get_ros_parameter(
            self,
            "wheel.tread",
        )
        # 左右車輪間距離 [m]
        # 本ファイルでは直接電流配分には使わないが、
        # 他のモジュールとのパラメータ互換性のため保持する。

        self.gear_ratio = get_ros_parameter(
            self,
            "wheel.gear_ratio",
        )
        # モータ回転数から車輪回転数への換算に使用するギア比

        self.publish_timer_loop_duration = get_ros_parameter(
            self,
            "publish_timer_loop_duration",
        )
        # CAN送信周期 [s]

        # ===============================
        # ヨー角速度PIDゲイン
        # 出力単位は差動電流 [A]
        # ===============================

        self.pd_dt = get_ros_parameter(self, "pd.dt")
        self.kp = get_ros_parameter(self, "pd.kp")
        self.ki = get_ros_parameter(self, "pd.ki")
        self.kd = get_ros_parameter(self, "pd.kd")

        self.max_output = get_ros_parameter(
            self,
            "pd.max_output",
        )
        self.min_output = get_ros_parameter(
            self,
            "pd.min_output",
        )

        # ===============================
        # 並進速度PIDゲイン
        # 出力単位は左右共通電流 [A]
        # ===============================

        self.kp_v = get_ros_parameter(self, "pd.kp_v")
        self.ki_v = get_ros_parameter(self, "pd.ki_v")
        self.kd_v = get_ros_parameter(self, "pd.kd_v")

        self.max_output_v = get_ros_parameter(
            self,
            "pd.max_output_v",
        )
        self.min_output_v = get_ros_parameter(
            self,
            "pd.min_output_v",
        )

    # ===============================
    # Twist受信コールバック
    # ===============================

    def twist_callback(self, msg: Twist) -> None:
        """
        上位ノードから速度・ヨー角速度の目標値を受け取る。

        外部との連携を維持するため、Twistの意味は変更しない。

            msg.linear.x  = v_ref     [m/s]
            msg.angular.z = omega_ref [rad/s]

        左右電流指令の計算は、このコールバックでは行わない。
        PID出力は別タイマーで更新されるため、CAN送信タイマー側で
        常に最新のPID出力から左右電流指令を組み立てる。
        """

        self.v_ref = float(msg.linear.x)
        self.omega_ref = float(msg.angular.z)

    # ===============================
    # IMU受信コールバック
    # ===============================

    def imu_callback(self, msg: Imu) -> None:
        """IMUから現在ヨー角速度を取得する。"""

        self.omega_now = float(msg.angular_velocity.z)

    # ===============================
    # CAN受信コールバック
    # ===============================

    def can_callback(self, msg: Frame) -> None:
        """
        モータドライバから回転数および電流フィードバックを受信する。

        0x711:
            data[0:4]  右モータRPM
            data[4:8]  左モータRPM

        0x712:
            学生版コードに合わせ、
            data[0:2]  右モータ電流値
            data[2:4]  左モータ電流値
            data[4:6]  右モータトルク(Nm × 100)
            data[6:8]  左モータトルク(Nm × 100)
            として読み取る。
        """

        if msg.id == self.CAN_RPM_FEEDBACK_ID:

            if self.last_can_msg is None:
                self.get_logger().info(
                    "Subscribed CAN RPM feedback frame."
                )

            self.last_can_msg = msg

        elif msg.id == self.CAN_CURRENT_FEEDBACK_ID:

            current_right = int.from_bytes(
                bytes(msg.data[0:2]),
                byteorder='little',
                signed=True,
            )

            current_left = int.from_bytes(
                bytes(msg.data[2:4]),
                byteorder='little',
                signed=True,
            )
            
            torque_right = int.from_bytes(
                bytes(msg.data[4:6]),
                byteorder='little',
                signed=True
	    )

            torque_left = int.from_bytes(
                bytes(msg.data[6:8]),
                byteorder='little',
                signed=True
            )

            # TPDO4の値が既に[A]なら、このままでよい。
            # 係数付きの整数値なら、ここで物理単位[A]へ変換する。
            self.current_right_feedback_debug = float(current_right)
            self.current_left_feedback_debug = float(current_left)
            self.torque_right_debug = float(torque_right)
            self.torque_left_debug = float(torque_left)   

    # ===============================
    # ヨー角速度PID制御
    # ===============================

    def control_yaw_rate_callback(self) -> None:
        """
        ヨー角速度偏差から左右差動電流成分を計算する。

            e_omega = omega_ref - omega

            i_omega =
                Kp_omega * e_omega
                + Ki_omega * integral(e_omega)
                + Kd_omega * d(e_omega)/dt

        正の i_omega は、

            i_right を増やす
            i_left  を減らす

        方向として配分される。
        実機のモータ配線とIMUの正方向によって符号が逆の場合は、
        calculate_current_commands()内の符号を反転する。
        """

        error = self.omega_ref - self.omega_now
        derivative = (error - self.e_prev) / self.pd_dt

        unsaturated_output = (
            self.kp * error
            + self.ki * self.ie
            + self.kd * derivative
        )

        output = float(np.clip(
            unsaturated_output,
            self.min_output,
            self.max_output,
        ))

        # 条件付き積分による簡易アンチワインドアップ。
        # 飽和していない場合、または誤差が飽和を解消する向きの場合のみ
        # 積分状態を更新する。
        saturation_high = unsaturated_output > self.max_output
        saturation_low = unsaturated_output < self.min_output

        if (
            (not saturation_high and not saturation_low)
            or (saturation_high and error < 0.0)
            or (saturation_low and error > 0.0)
        ):
            self.ie += error * self.pd_dt

        self.u_pd = output
        self.e_prev = error

    # ===============================
    # 並進速度PID制御
    # ===============================

    def control_velocity_callback(self) -> None:
        """
        並進速度偏差から左右共通電流成分を計算する。

            e_v = v_ref - v

            i_v =
                Kp_v * e_v
                + Ki_v * integral(e_v)
                + Kd_v * d(e_v)/dt

        i_v は左右モータに同符号で加えられ、
        主に車体の前後加速度を生成する。
        """

        velocity_now = self.calc_linear_velocity_from_can()
        error = self.v_ref - velocity_now
        derivative = (error - self.V_e_prev) / self.pd_dt

        unsaturated_output = (
            self.kp_v * error
            + self.ki_v * self.V_ie
            + self.kd_v * derivative
        )

        output = float(np.clip(
            unsaturated_output,
            self.min_output_v,
            self.max_output_v,
        ))

        saturation_high = unsaturated_output > self.max_output_v
        saturation_low = unsaturated_output < self.min_output_v

        if (
            (not saturation_high and not saturation_low)
            or (saturation_high and error < 0.0)
            or (saturation_low and error > 0.0)
        ):
            self.V_ie += error * self.pd_dt

        self.u_V = output
        self.V_e_prev = error
        self.V_e_debug = float(error)

    # ===============================
    # PID出力から左右電流指令を生成
    # ===============================

    def calculate_current_commands(self) -> Tuple[float, float]:
        """
        並進速度PID出力とヨー角速度PID出力から、
        左右モータの指令電流[A]を生成する。

        並進成分:
            i_v = u_V

        旋回成分:
            i_omega = u_pd

        電流配分:
            i_right = i_v + i_omega
            i_left  = i_v - i_omega

        左右いずれかがドライバ上限50 Aを超えた場合は、
        両電流を同じ倍率で縮小する。

        個別クリップではなく同一倍率で縮小する理由は、
        並進成分と旋回成分の比を可能な限り維持するためである。
        """

        common_current = float(self.u_V)
        differential_current = float(self.u_pd)

        current_left = common_current - differential_current
        current_right = common_current + differential_current

        max_abs_current = max(
            abs(current_left),
            abs(current_right),
        )

        if max_abs_current > self.DRIVER_CURRENT_LIMIT_AMP:
            scale = (
                self.DRIVER_CURRENT_LIMIT_AMP
                / max_abs_current
            )
            current_left *= scale
            current_right *= scale

        return float(current_left), float(current_right)

    # ===============================
    # 電流[A]からCAN指令整数値へ変換
    # ===============================

    @classmethod
    def current_to_can_command(cls, current_amp: float) -> int:
        """
        指令電流[A]をモータドライバへ送る整数値へ変換する。

            I_cmd [A] = raw / 1000 * 50

        よって、

            raw = I_cmd [A] * 20

        となる。
        """

        limited_current = float(np.clip(
            current_amp,
            -cls.DRIVER_CURRENT_LIMIT_AMP,
            cls.DRIVER_CURRENT_LIMIT_AMP,
        ))

        raw_command = round(
            limited_current * cls.AMP_TO_COMMAND_GAIN
        )

        return int(np.clip(
            raw_command,
            -cls.DRIVER_COMMAND_FULL_SCALE,
            cls.DRIVER_COMMAND_FULL_SCALE,
        ))

    # ===============================
    # CAN整数値を4byteへ変換
    # ===============================

    @staticmethod
    def toCanCmd(command_value: int) -> List[int]:
        """
        符号付き整数値を4byte little endianへ変換する。

        この関数は物理量の換算を行わない。
        速度制御モードではRPM指令値、
        電流制御モードでは電流指令のraw値を格納する。
        """

        byte_data = int(command_value).to_bytes(
            4,
            byteorder="little",
            signed=True,
        )

        return list(byte_data)

    # ===============================
    # CAN送信タイマー
    # ===============================

    def publish_canframe_callback(self) -> None:
        """
        最新のPID出力から左右電流指令を生成し、CAN送信する。

        学生版ではTwist受信時にCANデータを更新していたが、
        PID出力は別タイマーで変化するため、その構造では
        新しいTwistが届くまでCAN指令値が更新されない。

        本実装ではCAN送信周期ごとに最新のu_V, u_pdから
        左右電流を再計算する。
        """

        current_left, current_right = (
            self.calculate_current_commands()
        )

        self.current_left_cmd = current_left
        self.current_right_cmd = current_right

        raw_left = self.current_to_can_command(current_left)
        raw_right = self.current_to_can_command(current_right)

        # 既存仕様を維持し、右4byte + 左4byteの順に格納する。
        cmd_right = self.toCanCmd(raw_right)
        cmd_left = self.toCanCmd(raw_left)

        self.frame_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        self.frame_msg.data = cmd_right + cmd_left

        self.can_pub.publish(self.frame_msg)
        self.publish_debug_state()

    # ===============================
    # CANから現在速度計算
    # ===============================

    def calc_linear_velocity_from_can(self) -> float:
        """
        0x711で受信した左右モータRPMの平均から、
        車体並進速度[m/s]を算出する。
        """

        if self.last_can_msg is None:
            return 0.0

        data = self.last_can_msg.data

        rpm_right = int.from_bytes(
            bytes(data[0:4]),
            byteorder='little',
            signed=True,
        )

        rpm_left = int.from_bytes(
            bytes(data[4:8]),
            byteorder='little',
            signed=True,
        )

        rpm_avg = 0.5 * (rpm_right + rpm_left)

        self.rpm_right_debug = float(rpm_right)
        self.rpm_left_debug = float(rpm_left)
        self.rpm_avg_debug = float(rpm_avg)

        wheel_radius = self.diameter * 0.5

        velocity = (
            rpm_avg
            * 2.0
            * np.pi
            / 60.0
            * wheel_radius
            / self.gear_ratio
        )

        self.v_now_debug = float(velocity)

        return float(velocity)

    # ===============================
    # デバッグ情報publish
    # ===============================

    def publish_debug_state(self) -> None:
        """
        制御内部状態を既存デバッグトピックへpublishする。

        data:
            [0]  u_V                         左右共通電流成分 [A]
            [1]  u_pd                        左右差動電流成分 [A]
            [2]  omega_ref                   目標ヨー角速度 [rad/s]
            [3]  v_ref                       目標並進速度 [m/s]
            [4]  current_right_cmd           右指令電流 [A]
            [5]  current_left_cmd            左指令電流 [A]
            [6]  rpm_right                   右モータRPM
            [7]  rpm_left                    左モータRPM
            [8]  rpm_avg                     左右平均RPM
            [9]  v_now                       現在速度 [m/s]
            [10] velocity_error              速度偏差 [m/s]
            [11] current_right_feedback      ドライバ受信値
            [12] current_left_feedback       ドライバ受信値
            [13] torque_right            : 右モータトルク(Nm×100)
            [14] torque_left             : 左モータトルク(Nm×100)
        """

        debug_msg = Float32MultiArray()

        debug_msg.data = [
            float(self.u_V),
            float(self.u_pd),
            float(self.omega_ref),
            float(self.v_ref),
            float(self.current_right_cmd),
            float(self.current_left_cmd),
            float(self.rpm_right_debug),
            float(self.rpm_left_debug),
            float(self.rpm_avg_debug),
            float(self.v_now_debug),
            float(self.V_e_debug),
            float(self.current_right_feedback_debug),
            float(self.current_left_feedback_debug),
            float(self.torque_right_debug),
            float(self.torque_left_debug),
        ]

        self.debug_pub.publish(debug_msg)

    # ===============================
    # 停止・終了処理
    # ===============================

    def stop_vehicle(self) -> None:
        """
        ノード終了時にゼロ電流指令を送信する。

        これは制御停止時の安全な出力初期化であり、
        車両を閉ループで制動する処理ではない。
        """

        self.v_ref = 0.0
        self.omega_ref = 0.0

        self.u_V = 0.0
        self.u_pd = 0.0

        self.V_ie = 0.0
        self.ie = 0.0

        zero_command = self.toCanCmd(0)

        self.frame_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        self.frame_msg.data = zero_command + zero_command

        self.can_pub.publish(self.frame_msg)


# ===============================
# メイン関数
# ===============================

def main(args=None) -> None:

    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)

    except KeyboardInterrupt:
        motor_controller.get_logger().info(
            "KeyboardInterrupt received. Sending zero-current command."
        )

    finally:
        motor_controller.stop_vehicle()
        motor_controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
