#!/usr/bin/env python
# Pythonスクリプトを実行するときに使用するPythonインタプリタを指定する
import rclpy
# ROS 2のPythonクライアントライブラリをインポートする
import math
# 円周率πなどの数学関数を使用するためにインポートする
import numpy as np
# 配列計算などを行うNumPyをインポートする
from rclpy.node import Node
# ROS 2ノードを作成するためのNodeクラスをインポートする
from geometry_msgs.msg import Twist
# 速度指令などに使用するTwistメッセージ型をインポートする
from can_msgs.msg import Frame
# CANフレームを扱うFrameメッセージ型をインポートする
from std_msgs.msg import Float32MultiArray
# debug/controller_stateで使用するFloat32MultiArray型をインポートする
from typing import List
# List型を型ヒントとして使用するためにインポートする
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# CAN通信で使用するQoS設定をインポートする

# ==========================================================
# CAN通信用QoS設定
# ==========================================================

qos_can = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    # CAN通信ではBEST_EFFORTを使用する
    # 少しデータが欠損しても通信速度を優先する

    history=HistoryPolicy.KEEP_LAST,
    # 最新のメッセージを保持する

    depth=10
    # 最新10個のメッセージを保持する
)

class MotorController(Node):
    # MotorControllerというROS 2ノードのクラスを定義する

    def __init__(self):
        # MotorControllerクラスが生成されたときに実行される初期化処理

        super().__init__('motor_controller')
        # ROS 2のNodeクラスを初期化する
        # ノード名をmotor_controllerに設定する

        # ======================================================
        # ROSパラメータ
        # ======================================================

        self.declare_parameter('wheel.tread')
        # 車輪間距離のROSパラメータを宣言する

        self.declare_parameter('wheel.diameter')
        # 車輪直径のROSパラメータを宣言する

        self.declare_parameter('wheel.gear_ratio')
        # ギヤ比のROSパラメータを宣言する

        self.declare_parameter('publish_timer_loop_duration')
        # CAN送信周期のROSパラメータを宣言する


        self.tread = self.get_parameter(
            'wheel.tread'
        ).get_parameter_value().double_value
        # wheel.treadの値を取得する


        self.diameter = self.get_parameter(
            'wheel.diameter'
        ).get_parameter_value().double_value
        # wheel.diameterの値を取得する


        self.gear_ratio = self.get_parameter(
            'wheel.gear_ratio'
        ).get_parameter_value().double_value
        # wheel.gear_ratioの値を取得する

        publish_timer_loop_duration = self.get_parameter(
            'publish_timer_loop_duration'
        ).get_parameter_value().double_value
        # CAN送信周期を取得する

        # ======================================================
        # Publisher / Subscriber
        # ======================================================

        buffer_size = 10
        # 通常のSubscriber / Publisherのキューサイズを10に設定する

        self.twist_sub = self.create_subscription(
            Twist,
            '/aiformula_control/twist_mux/cmd_vel',
            self.twist_callback,
            buffer_size
        )
        # sub_speed_commandを購読する
        #
        # Twistを受信するとtwist_callback()を実行する

        self.can_sub = self.create_subscription(
            Frame,
            'sub_can',
            self.can_callback,
            qos_can
        )
        # sub_canを購読する
        #
        # CAN Frameを受信するとcan_callback()を実行する
        # ここでモータドライバから
        # RPM、電流、トルクを受信する

        self.can_pub = self.create_publisher(
            Frame,
            'pub_can',
            buffer_size
        )
        # pub_canへCAN Frameを送信するPublisherを作成する

        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            'debug/controller_state',
            18
        )
        # debug/controller_stateへ
        # 制御状態をFloat32MultiArrayとしてpublishする

        # ======================================================
        # CAN送信タイマー
        # ======================================================

        self.publish_timer = self.create_timer(
            publish_timer_loop_duration,
            self.publish_canframe_callback
        )
        # 指定された周期でCANフレームを送信する

        # ======================================================
        # CAN送信フレーム
        # ======================================================

        self.frame_msg = Frame()
        # CAN送信用のFrameメッセージを生成する

        self.frame_msg.header.frame_id = "can0"
        # CANインターフェースをcan0に設定する

        self.frame_msg.id = 0x210
        # モータ指令用CAN IDを0x210に設定する

        self.frame_msg.dlc = 8
        # CANデータ長を8byteに設定する

        # ======================================================
        # CAN受信データ保存用変数
        # ======================================================

        self.last_can_msg = None
        # 最後に受信したRPM用CANフレームを保存する

        self.rpm_right_debug = 0.0
        # 右モータRPM保存用

        self.rpm_left_debug = 0.0
        # 左モータRPM保存用

        self.rpm_avg_debug = 0.0
        # 左右モータ平均RPM保存用

        self.v_now_debug = 0.0
        # CANから計算した現在速度保存用

        self.V_e_debug = 0.0
        # 速度誤差保存用

        self.current_right_debug = 0.0
        # 右モータ電流保存用

        self.current_left_debug = 0.0
        # 左モータ電流保存用

        self.torque_right_debug = 0.0
        # 右モータトルク保存用

        self.torque_left_debug = 0.0
        # 左モータトルク保存用

    # ==========================================================
    # Twist受信コールバック
    # ==========================================================

    def twist_callback(self, msg):
        # sub_speed_commandからTwistを受信したときに実行される

        rpm = self.toRefRPM(
            msg.linear.x,
            msg.angular.z
        )
        # 車両速度とyaw角速度から
        # 左右モータの目標RPMを計算する

        cmd_right = self.toCanCmd(rpm[0])
        # 右モータRPMをCAN送信用4byteデータへ変換する


        cmd_left = self.toCanCmd(rpm[1])
        # 左モータRPMをCAN送信用4byteデータへ変換する


        can_data = cmd_right + cmd_left
        # 右4byte + 左4byte
        # 合計8byteのCANデータを作成する


        self.frame_msg.data = can_data
        # 作成したCANデータをFrameに格納する


    # ==========================================================
    # CAN受信コールバック
    # ==========================================================

    def can_callback(self, msg: Frame):
        # sub_canからCAN Frameを受信したときに実行される


        RPM_ID = 0x711
        # モータRPMを受信するCAN ID
        # 以前のPID版と同じ


        CURRENT_ID = 0x712
        # モータ電流・トルクを受信するCAN ID
        # 以前のPID版と同じ


        # ======================================================
        # RPM受信
        # ======================================================

        if msg.id == RPM_ID:
            # 受信したCAN IDが0x711ならRPMデータとして処理する


            if self.last_can_msg is None:
                # 初めてRPMフレームを受信した場合


                self.get_logger().info(
                    "Subscribe CAN RPM Frame !"
                )
                # RPM CANフレームを受信したことをログ表示する


            self.last_can_msg = msg
            # 最新のRPM CANフレームを保存する


            data = msg.data
            # CANデータ8byteを取得する


            rpm_right = int.from_bytes(
                bytes(data[0:4]),
                byteorder='little',
                signed=True
            )
            # CAN Byte0～Byte3から
            # 右モータRPMを復元する


            rpm_left = int.from_bytes(
                bytes(data[4:8]),
                byteorder='little',
                signed=True
            )
            # CAN Byte4～Byte7から
            # 左モータRPMを復元する


            rpm_avg = 0.5 * (
                rpm_right + rpm_left
            )
            # 左右モータRPMの平均値を計算する


            self.rpm_right_debug = float(rpm_right)
            # 右RPMを保存する


            self.rpm_left_debug = float(rpm_left)
            # 左RPMを保存する


            self.rpm_avg_debug = float(rpm_avg)
            # 平均RPMを保存する


            wheel_radius = self.diameter * 0.5
            # 車輪半径[m]を計算する


            v = (
                (rpm_avg * 2.0 * np.pi / 60.0)
                * wheel_radius
                / self.gear_ratio
            )
            # モータRPMから車輪角速度へ変換し、
            # 車輪半径を掛けて車両速度[m/s]を計算する
            #
            # rpm
            # ↓
            # rad/s
            # ↓
            # wheel speed[m/s]


            self.v_now_debug = float(v)
            # 現在速度を保存する


        # ======================================================
        # 電流・トルク受信
        # ======================================================

        elif msg.id == CURRENT_ID:
            # 受信したCAN IDが0x712なら
            # 電流・トルクデータとして処理する


            current_right = int.from_bytes(
                bytes(msg.data[0:2]),
                byteorder='little',
                signed=True
            )
            # Byte0～Byte1から右モータ電流を復元する


            current_left = int.from_bytes(
                bytes(msg.data[2:4]),
                byteorder='little',
                signed=True
            )
            # Byte2～Byte3から左モータ電流を復元する


            torque_right = int.from_bytes(
                bytes(msg.data[4:6]),
                byteorder='little',
                signed=True
            )
            # Byte4～Byte5から右モータトルクを復元する


            torque_left = int.from_bytes(
                bytes(msg.data[6:8]),
                byteorder='little',
                signed=True
            )
            # Byte6～Byte7から左モータトルクを復元する


            self.current_right_debug = float(
                current_right
            )
            # 右モータ電流を保存する


            self.current_left_debug = float(
                current_left
            )
            # 左モータ電流を保存する


            self.torque_right_debug = float(
                torque_right
            )
            # 右モータトルクを保存する


            self.torque_left_debug = float(
                torque_left
            )
            # 左モータトルクを保存する


    # ==========================================================
    # CAN送信
    # ==========================================================

    def publish_canframe_callback(self):
        # CAN送信タイマーによって周期的に実行される


        self.can_pub.publish(self.frame_msg)
        # pub_canへCANフレームをpublishする


    # ==========================================================
    # Velocity -> RPM Calc
    # ==========================================================

    def toRefRPM(
        self,
        linear_velocity,
        angular_velocity
    ):
        # 車両速度とyaw角速度から
        # 左右モータの目標RPMを計算する


        wheel_angular_velocities = np.array([
            (linear_velocity / (self.diameter * 0.5))
            + (self.tread / self.diameter)
            * angular_velocity,
            # 右車輪角速度[rad/s]


            (linear_velocity / (self.diameter * 0.5))
            - (self.tread / self.diameter)
            * angular_velocity
            # 左車輪角速度[rad/s]
        ])
        # 左右車輪の角速度を計算する


        minute_to_second = 60
        # 秒から分への変換係数


        rpm = wheel_angular_velocities * (
            minute_to_second / (2 * math.pi)
        )
        print(f"rpm: {rpm}")
        # 車輪角速度[rad/s]をRPMへ変換する


        rpm = rpm * self.gear_ratio
        # ギヤ比を掛けてモータ側RPMへ変換する


        # ======================================================
        # debug/controller_state
        # ======================================================

        u_V = 0.0
        # 今回のプログラムでは速度PIDを使用していないため0


        u_pd = 0.0
        # 今回のプログラムではヨーレートPIDを使用していないため0


        V_e = linear_velocity - self.v_now_debug
        # 目標速度とCANから取得した実速度との差を計算する
        #
        # V_e = V_ref - V_now


        debug_msg = Float32MultiArray()
        # debug/controller_state用メッセージを生成する


        debug_msg.data = [

            float(u_V),
            # [0] 速度PID出力


            float(u_pd),
            # [1] ヨーレートPID出力


            float(angular_velocity),
            # [2] 目標ヨーレート[rad/s]


            float(linear_velocity),
            # [3] 目標直進速度[m/s]


            float(wheel_angular_velocities[0]),
            # [4] 右車輪角速度[rad/s]


            float(wheel_angular_velocities[1]),
            # [5] 左車輪角速度[rad/s]


            float(self.rpm_right_debug),
            # [6] CANから受信した右モータRPM


            float(self.rpm_left_debug),
            # [7] CANから受信した左モータRPM


            float(self.rpm_avg_debug),
            # [8] CANから受信した左右平均RPM


            float(self.v_now_debug),
            # [9] CANから計算した現在速度[m/s]


            float(V_e),
            # [10] 速度誤差[m/s]


            float(self.current_right_debug),
            # [11] CANから受信した右モータ電流


            float(self.current_left_debug),
            # [12] CANから受信した左モータ電流


            float(self.torque_right_debug),
            # [13] CANから受信した右モータトルク


            float(self.torque_left_debug)
            # [14] CANから受信した左モータトルク
        ]
        self.debug_pub.publish(debug_msg)
        # debug/controller_stateへデバッグデータをpublishする
        return rpm.tolist()
        # 計算した左右モータRPMをPythonのlistとして返す


    # ==========================================================
    # RPM -> CANデータ変換
    # ==========================================================

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        # RPMをCAN送信用のバイトデータに変換する


        rounded = round(rpm)
        # RPMを四捨五入して整数にする


        bytes = rounded.to_bytes(
            4,
            "little",
            signed=True
        )
        # RPM整数値を4byteのlittle endianへ変換する
        # signed=Trueなので正負のRPMを扱える


        return list(bytes)
        # bytesをlistへ変換して返す


# ==========================================================
# メイン関数
# ==========================================================

def main(args=None):
    # ROS 2ノードを起動するためのmain関数
    rclpy.init(args=args)
    # ROS 2通信を初期化する
    motor_controller = MotorController()
    # MotorControllerノードを生成する
    rclpy.spin(motor_controller)
    # ノードを継続的に実行する
    motor_controller.destroy_node()
    # ノードを破棄する
    rclpy.shutdown()
    # ROS 2通信を終了する


# ==========================================================
# Python実行時エントリ
# ==========================================================

if __name__ == '__main__':
    # このPythonファイルが直接実行された場合
    main()
    # main関数を実行する
