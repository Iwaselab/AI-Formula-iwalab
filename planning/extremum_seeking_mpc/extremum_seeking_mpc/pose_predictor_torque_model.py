#!/usr/bin/env python3
"""
pose_predictor_torque_model.py

既存のPosePredictorと同じ外部インターフェースを保ちながら、
曲率候補から将来軌道を予測する内部モデルを、

    「一定速度で、指定曲率が瞬時に実現する円弧モデル」

から、

    「左右モータ電流 → 車輪トルク → 並進加速度・ヨー角加速度
      → 速度・ヨー角速度 → 車体位置・姿勢」

という動的モデルへ変更した実装。

このファイルの役割
------------------
PathOptimizerが出力する曲率列

    kappa = [kappa_0, kappa_1, ..., kappa_N-1]

は、「実現したい目標曲率列」として扱う。

PosePredictorは、その目標曲率を現在の車両状態と
モータ電流・トルク制約の下で実行したとき、

    X, Y, yaw, V, omega

がどのように変化するかを予測する。

予測された軌道上のseek_positionsが、
ObjectRiskCalculatorおよびRoadRiskCalculatorへ渡されるため、
車両ダイナミクスの影響は最終的にリスク値を介して
PathOptimizerの曲率更新へ反映される。

重要
----
このクラス自身が曲率を最適化するわけではない。
しかし、

    曲率候補
      → 動的に実現可能な予測軌道
      → リスク
      → 次の曲率候補

という閉ループを構成するため、
本クラスの動的モデルは選択される曲率列に直接影響する。

既存ノードとの互換性
--------------------
以下のpublic method名と引数は既存コードに合わせて維持する。

    predict_relative_ego_positions(curvatures)
    predict_pose(curvature, horizon_time)
    predict_relative_seek_positions(ego_positions)
    predict_absolute_seek_positions(
        ego_positions,
        relative_seek_positions
    )

また、Odometryの購読トピック sub_odom も変更しない。
"""

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node

from common_python.get_ros_parameter import get_ros_parameter
from .util import Position2d, Pose, Velocity


@dataclass
class DynamicState:
    """
    予測計算内部で使用する車両状態。

    x, y:
        予測開始時の車体座標系を原点とした位置 [m]

    yaw:
        予測開始時の車体姿勢を0としたヨー角 [rad]

    linear_velocity:
        車体前後方向速度 V [m/s]

    yaw_rate:
        ヨー角速度 omega [rad/s]
    """

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

        # Odometry受信前でも安全にゼロ状態から計算できるよう初期化する。
        self.ego_current_velocity = Velocity(
            linear=0.0,
            angular=0.0,
        )
        self.has_received_odometry = False

        self.horizon_times = np.asarray(
            horizon_times,
            dtype=float,
        )

        if self.horizon_times.ndim != 1:
            raise ValueError(
                "horizon_times must be a one-dimensional sequence."
            )

        if len(self.horizon_times) == 0:
            raise ValueError(
                "horizon_times must contain at least one prediction time."
            )

        if np.any(self.horizon_times <= 0.0):
            raise ValueError(
                "All horizon_times must be positive."
            )

        if np.any(np.diff(self.horizon_times) <= 0.0):
            raise ValueError(
                "horizon_times must be strictly increasing."
            )

        # 各予測区間の長さ。
        # 例: horizon_times = [0.5, 1.0, 1.5]
        #     horizon_durations = [0.5, 0.5, 0.5]
        self.horizon_durations = np.diff(
            self.horizon_times,
            prepend=0.0,
        )

        self.horizon_length = len(self.horizon_times)

        self.seek_y_positions = np.asarray(
            seek_y_positions,
            dtype=float,
        )

        if self.seek_y_positions.shape[0] != self.horizon_length:
            raise ValueError(
                "The number of seek-point sets must match "
                "the number of horizon steps."
            )

        # 直前の予測で得られた姿勢角。
        # seek pointの横方向オフセットを車体姿勢に応じて回転するために使う。
        self.last_predicted_yaws = np.zeros(
            self.horizon_length,
            dtype=float,
        )

        # デバッグや後続処理の確認用として予測状態列も保持する。
        self.last_predicted_states = [
            DynamicState()
            for _ in range(self.horizon_length)
        ]

    # ===============================
    # ROSパラメータ
    # ===============================

    def init_parameters(self, node: Node) -> None:
        """
        既存パラメータと、動的予測に必要な追加パラメータを取得する。

        既存パラメータ:
            curvature_radius_maximum
            planned_speed
            velocity_control.deceleration_angle_maximum
            velocity_control.deceleration_gain

        追加パラメータ:
            prediction.integration_dt
            prediction.velocity_response_time
            prediction.yaw_rate_response_time
            prediction.linear_viscous_resistance
            prediction.yaw_viscous_resistance
            vehicle.mass
            vehicle.yaw_inertia
            wheel.diameter
            wheel.tread
            motor.wheel_torque_per_amp
            motor.current_limit_amp

        vehicle.mass等の物理パラメータは、
        実機値をYAMLへ明示的に設定する必要がある。
        """

        self.curvature_radius_maximum = get_ros_parameter(
            node,
            "curvature_radius_maximum",
        )

        planned_speed_parameter = get_ros_parameter(
            node,
            "planned_speed",
        )
        self.planned_speed = self._first_scalar(
            planned_speed_parameter,
            "planned_speed",
        )

        self.deceleration_angle_maximum = get_ros_parameter(
            node,
            "velocity_control.deceleration_angle_maximum",
        )

        self.deceleration_gain = get_ros_parameter(
            node,
            "velocity_control.deceleration_gain",
        )

        # 以下は本改編で追加するパラメータ。
        #
        # declare_parameter()を用いるため、
        # YAMLに値があればその値が採用され、
        # 設定がない場合はdefault_valueが用いられる。
        self.integration_dt = self._get_parameter_or_default(
            node,
            "prediction.integration_dt",
            0.01,
        )

        # 目標速度へ近づく時間スケール [s]。
        # 実際の下位速度制御系のステップ応答から調整する。
        self.velocity_response_time = (
            self._get_parameter_or_default(
                node,
                "prediction.velocity_response_time",
                0.5,
            )
        )

        # 目標ヨー角速度へ近づく時間スケール [s]。
        # 実際の下位ヨー角速度制御系の応答から調整する。
        self.yaw_rate_response_time = (
            self._get_parameter_or_default(
                node,
                "prediction.yaw_rate_response_time",
                0.3,
            )
        )

        # 並進方向の粘性抵抗係数 [N/(m/s)]。
        # 初期検証では0としてよいが、走行データから同定可能。
        self.linear_viscous_resistance = (
            self._get_parameter_or_default(
                node,
                "prediction.linear_viscous_resistance",
                0.0,
            )
        )

        # ヨー方向の粘性抵抗係数 [N m/(rad/s)]。
        self.yaw_viscous_resistance = (
            self._get_parameter_or_default(
                node,
                "prediction.yaw_viscous_resistance",
                0.0,
            )
        )

        # 車両質量 [kg]
        self.vehicle_mass = self._get_parameter_or_default(
            node,
            "vehicle.mass",
            -1.0,
        )

        # 車体のヨー慣性モーメント [kg m^2]
        self.vehicle_yaw_inertia = (
            self._get_parameter_or_default(
                node,
                "vehicle.yaw_inertia",
                -1.0,
            )
        )

        # 車輪直径 [m]
        self.wheel_diameter = self._get_parameter_or_default(
            node,
            "wheel.diameter",
            -1.0,
        )

        # 左右車輪間距離 [m]
        self.wheel_tread = self._get_parameter_or_default(
            node,
            "wheel.tread",
            -1.0,
        )

        # モータ電流1 A当たりの実効ホイール軸トルク [N m/A]
        #
        # モータドライバ表示値から0.27 N m/A程度が観測されているが、
        # それがホイール軸換算値であることを確認した上で設定する。
        self.wheel_torque_per_amp = (
            self._get_parameter_or_default(
                node,
                "motor.wheel_torque_per_amp",
                -1.0,
            )
        )

        # 片側モータの電流上限 [A]
        self.current_limit_amp = (
            self._get_parameter_or_default(
                node,
                "motor.current_limit_amp",
                50.0,
            )
        )

        self._validate_physical_parameters()

    @staticmethod
    def _get_parameter_or_default(
        node: Node,
        name: str,
        default_value,
    ):
        """
        ROSパラメータを安全に宣言・取得する。

        YAMLにoverrideがあれば、その値が採用される。
        """

        if not node.has_parameter(name):
            node.declare_parameter(name, default_value)

        return node.get_parameter(name).value

    @staticmethod
    def _first_scalar(value, parameter_name: str) -> float:
        """
        planned_speedがスカラーまたは配列のどちらでも、
        最初の値をfloatとして取得する。
        """

        array = np.asarray(value, dtype=float).reshape(-1)

        if array.size == 0:
            raise ValueError(
                f"{parameter_name} must not be empty."
            )

        return float(array[0])

    def _validate_physical_parameters(self) -> None:
        """動的モデルに必要な物理パラメータを検査する。"""

        positive_parameters = {
            "prediction.integration_dt": self.integration_dt,
            "prediction.velocity_response_time":
                self.velocity_response_time,
            "prediction.yaw_rate_response_time":
                self.yaw_rate_response_time,
            "vehicle.mass": self.vehicle_mass,
            "vehicle.yaw_inertia": self.vehicle_yaw_inertia,
            "wheel.diameter": self.wheel_diameter,
            "wheel.tread": self.wheel_tread,
            "motor.wheel_torque_per_amp":
                self.wheel_torque_per_amp,
            "motor.current_limit_amp":
                self.current_limit_amp,
        }

        invalid = [
            name
            for name, value in positive_parameters.items()
            if float(value) <= 0.0
        ]

        if invalid:
            invalid_text = ", ".join(invalid)
            raise ValueError(
                "The following parameters must be positive: "
                f"{invalid_text}"
            )

        if self.curvature_radius_maximum <= 0.0:
            raise ValueError(
                "curvature_radius_maximum must be positive."
            )

    # ===============================
    # ROS接続
    # ===============================

    def init_connections(
        self,
        node: Node,
        buffer_size: int,
    ) -> None:

        self.actual_speed_sub = node.create_subscription(
            Odometry,
            'sub_odom',
            self.odometry_callback,
            buffer_size,
        )

    def odometry_callback(
        self,
        odom_msg: Odometry,
    ) -> None:
        """
        現在の車体前後速度とヨー角速度を保存する。

        従来コードではlinear.xとlinear.yのノルムを使用していたが、
        それでは後退時にも速度が正となる。

        加速・制動・後退を区別するため、
        車体前後方向速度としてlinear.xを符号付きで使用する。
        """

        linear_velocity = float(
            odom_msg.twist.twist.linear.x
        )

        angular_velocity = float(
            odom_msg.twist.twist.angular.z
        )

        self.ego_current_velocity = Velocity(
            linear=linear_velocity,
            angular=angular_velocity,
        )

        self.has_received_odometry = True

    # ===============================
    # 車両動的モデル
    # ===============================

    def _normalize_curvatures(
        self,
        curvatures: Sequence[float],
    ) -> np.ndarray:
        """
        曲率列を予測ホライズン長へ合わせる。

        要素が不足する場合は最後の曲率を保持する。
        要素が多い場合は必要数だけ使用する。
        """

        curvature_array = np.asarray(
            curvatures,
            dtype=float,
        ).reshape(-1)

        if curvature_array.size == 0:
            curvature_array = np.zeros(
                self.horizon_length,
                dtype=float,
            )

        if curvature_array.size < self.horizon_length:
            curvature_array = np.pad(
                curvature_array,
                (
                    0,
                    self.horizon_length
                    - curvature_array.size,
                ),
                mode='edge',
            )

        return curvature_array[:self.horizon_length]

    def _effective_curvature(
        self,
        curvature: float,
    ) -> float:
        """
        非常に小さい曲率を直進として扱う。

        curvature_radius_maximumは、
        これより大きい旋回半径を直進近似するための値である。

            |kappa| <= 1 / R_max

        のときkappa = 0とする。
        """

        curvature_deadband = (
            1.0 / self.curvature_radius_maximum
        )

        if abs(curvature) <= curvature_deadband:
            return 0.0

        return float(curvature)

    def _calculate_target_linear_velocity(
        self,
        curvature: float,
        current_velocity: float,
        interval_duration: float,
    ) -> float:
        """
        現行extremum_seeking_mpcと同じ考え方で、
        旋回が大きい場合に目標速度を下げる。

        予測区間内の概算ヨー角変化を、

            Delta_yaw ~= V * kappa * Delta_t

        とし、閾値を超えた分だけ目標速度を下げる。

        ここで得られるのは「瞬時に実現する速度」ではなく、
        下位速度制御系が追従すべき目標速度である。
        実速度はトルクモデルを通じて徐々に変化する。
        """

        nominal_velocity = self.planned_speed

        estimated_yaw_change = abs(
            current_velocity
            * curvature
            * interval_duration
        )

        excess_yaw_angle = (
            estimated_yaw_change
            - self.deceleration_angle_maximum
        )

        if excess_yaw_angle <= 0.0:
            return nominal_velocity

        nominal_sign = (
            1.0
            if nominal_velocity >= 0.0
            else -1.0
        )

        reduced_speed_magnitude = max(
            abs(nominal_velocity)
            - excess_yaw_angle
            * self.deceleration_gain,
            0.0,
        )

        return (
            nominal_sign
            * reduced_speed_magnitude
        )

    def _calculate_desired_accelerations(
        self,
        state: DynamicState,
        curvature: float,
        interval_duration: float,
    ) -> Tuple[float, float]:
        """
        目標曲率を実現するための目標並進加速度と
        目標ヨー角加速度を計算する。

        目標ヨー角速度:
            omega_ref = V * kappa

        ただし、V=0付近で完全にヨー指令が消えないよう、
        現在速度と目標速度のうち絶対値の大きい方を
        曲率からヨー角速度へ変換する基準速度とする。

        目標加速度は、所定の応答時間で目標値へ近づく
        一次遅れ型の内部モデルとして、

            a_des =
                (V_ref - V) / T_v

            alpha_des =
                (omega_ref - omega) / T_omega

        とする。

        その後、逆動力学によって必要電流を求め、
        電流制限を適用するため、実現不能な加速度は
        自動的に小さくなる。
        """

        target_velocity = (
            self._calculate_target_linear_velocity(
                curvature,
                state.linear_velocity,
                interval_duration,
            )
        )

        curvature_reference_velocity = (
            state.linear_velocity
            if abs(state.linear_velocity)
            >= abs(target_velocity)
            else target_velocity
        )

        target_yaw_rate = (
            curvature_reference_velocity
            * curvature
        )

        desired_linear_acceleration = (
            target_velocity
            - state.linear_velocity
        ) / self.velocity_response_time

        desired_yaw_acceleration = (
            target_yaw_rate
            - state.yaw_rate
        ) / self.yaw_rate_response_time

        return (
            float(desired_linear_acceleration),
            float(desired_yaw_acceleration),
        )

    def _desired_accelerations_to_currents(
        self,
        state: DynamicState,
        desired_linear_acceleration: float,
        desired_yaw_acceleration: float,
    ) -> Tuple[float, float]:
        """
        目標加速度から左右モータ電流を逆動力学で求める。

        車輪半径:
            r

        実効ホイール軸トルク係数:
            K_tau [N m/A]

        左右電流を、

            i_R = i_common + i_diff
            i_L = i_common - i_diff

        とする。

        並進運動:
            M * a
              = (tau_R + tau_L) / r
                - c_v * V

        ヨー運動:
            I_z * alpha
              = b / (2r) * (tau_R - tau_L)
                - c_omega * omega

        tau_R = K_tau * i_R
        tau_L = K_tau * i_L

        より、

            i_common
              = r / (2 K_tau)
                * (M a + c_v V)

            i_diff
              = r / (b K_tau)
                * (I_z alpha + c_omega omega)

        を得る。
        """

        wheel_radius = 0.5 * self.wheel_diameter

        common_current = (
            wheel_radius
            * (
                self.vehicle_mass
                * desired_linear_acceleration
                + self.linear_viscous_resistance
                * state.linear_velocity
            )
            / (
                2.0
                * self.wheel_torque_per_amp
            )
        )

        differential_current = (
            wheel_radius
            * (
                self.vehicle_yaw_inertia
                * desired_yaw_acceleration
                + self.yaw_viscous_resistance
                * state.yaw_rate
            )
            / (
                self.wheel_tread
                * self.wheel_torque_per_amp
            )
        )

        current_right = (
            common_current
            + differential_current
        )

        current_left = (
            common_current
            - differential_current
        )

        # 左右いずれかが電流上限を超えた場合は、
        # 両者を同じ倍率で縮小する。
        #
        # 個別にclipすると並進成分と旋回成分の比が変わり、
        # 予測された車体運動が不自然に変化するためである。
        max_abs_current = max(
            abs(current_left),
            abs(current_right),
        )

        if max_abs_current > self.current_limit_amp:
            scale = (
                self.current_limit_amp
                / max_abs_current
            )
            current_left *= scale
            current_right *= scale

        return (
            float(current_left),
            float(current_right),
        )

    def _currents_to_actual_accelerations(
        self,
        state: DynamicState,
        current_left: float,
        current_right: float,
    ) -> Tuple[float, float]:
        """
        電流制限後の左右電流から、
        実際に予測へ使用する並進加速度と
        ヨー角加速度を計算する。
        """

        wheel_radius = 0.5 * self.wheel_diameter

        torque_left = (
            self.wheel_torque_per_amp
            * current_left
        )

        torque_right = (
            self.wheel_torque_per_amp
            * current_right
        )

        drive_force = (
            torque_right
            + torque_left
        ) / wheel_radius

        linear_resistance_force = (
            self.linear_viscous_resistance
            * state.linear_velocity
        )

        linear_acceleration = (
            drive_force
            - linear_resistance_force
        ) / self.vehicle_mass

        yaw_moment = (
            self.wheel_tread
            / (
                2.0
                * wheel_radius
            )
            * (
                torque_right
                - torque_left
            )
        )

        yaw_resistance_moment = (
            self.yaw_viscous_resistance
            * state.yaw_rate
        )

        yaw_acceleration = (
            yaw_moment
            - yaw_resistance_moment
        ) / self.vehicle_yaw_inertia

        return (
            float(linear_acceleration),
            float(yaw_acceleration),
        )

    @staticmethod
    def _integrate_state(
        state: DynamicState,
        linear_acceleration: float,
        yaw_acceleration: float,
        dt: float,
    ) -> DynamicState:
        """
        一区間の状態を数値積分する。

        速度とヨー角速度は台形則相当の中点値を用い、
        位置と姿勢を更新する。

            V_next =
                V + a dt

            omega_next =
                omega + alpha dt

            V_mid =
                (V + V_next) / 2

            omega_mid =
                (omega + omega_next) / 2

            yaw_next =
                yaw + omega_mid dt

            X_next =
                X + V_mid cos(yaw_mid) dt

            Y_next =
                Y + V_mid sin(yaw_mid) dt
        """

        next_linear_velocity = (
            state.linear_velocity
            + linear_acceleration
            * dt
        )

        next_yaw_rate = (
            state.yaw_rate
            + yaw_acceleration
            * dt
        )

        middle_linear_velocity = 0.5 * (
            state.linear_velocity
            + next_linear_velocity
        )

        middle_yaw_rate = 0.5 * (
            state.yaw_rate
            + next_yaw_rate
        )

        next_yaw = (
            state.yaw
            + middle_yaw_rate
            * dt
        )

        middle_yaw = 0.5 * (
            state.yaw
            + next_yaw
        )

        next_x = (
            state.x
            + middle_linear_velocity
            * np.cos(middle_yaw)
            * dt
        )

        next_y = (
            state.y
            + middle_linear_velocity
            * np.sin(middle_yaw)
            * dt
        )

        return DynamicState(
            x=float(next_x),
            y=float(next_y),
            yaw=float(next_yaw),
            linear_velocity=float(
                next_linear_velocity
            ),
            yaw_rate=float(next_yaw_rate),
        )

    def _simulate_interval(
        self,
        initial_state: DynamicState,
        curvature: float,
        interval_duration: float,
    ) -> DynamicState:
        """
        一つの予測区間について動的モデルを積分する。

        1. 目標曲率から目標ヨー角速度を計算
        2. 目標速度・目標ヨー角速度への必要加速度を計算
        3. 逆動力学で左右電流を計算
        4. 電流上限を適用
        5. 制限後電流から実加速度を再計算
        6. 状態を積分

        これにより、曲率候補が大きくても、
        ヨー慣性や電流上限のために実曲率が直ちには
        目標値へ達しない現象が予測軌道へ反映される。
        """

        if interval_duration <= 0.0:
            return DynamicState(**vars(initial_state))

        effective_curvature = (
            self._effective_curvature(curvature)
        )

        num_steps = max(
            1,
            int(np.ceil(
                interval_duration
                / self.integration_dt
            )),
        )

        dt = interval_duration / num_steps

        state = DynamicState(**vars(initial_state))

        for _ in range(num_steps):

            (
                desired_linear_acceleration,
                desired_yaw_acceleration,
            ) = self._calculate_desired_accelerations(
                state,
                effective_curvature,
                interval_duration,
            )

            (
                current_left,
                current_right,
            ) = self._desired_accelerations_to_currents(
                state,
                desired_linear_acceleration,
                desired_yaw_acceleration,
            )

            (
                actual_linear_acceleration,
                actual_yaw_acceleration,
            ) = self._currents_to_actual_accelerations(
                state,
                current_left,
                current_right,
            )

            state = self._integrate_state(
                state,
                actual_linear_acceleration,
                actual_yaw_acceleration,
                dt,
            )

        return state

    # ===============================
    # 既存インターフェース互換関数
    # ===============================

    def predict_relative_ego_positions(
        self,
        curvatures: np.ndarray,
    ) -> np.ndarray:
        """
        曲率列を実行した場合の各ホライズン終端位置を返す。

        戻り値:
            shape = (horizon_length, 2)

            [
                [x_0, y_0],
                [x_1, y_1],
                ...
            ]

        各位置は予測開始時のbase_footprintを原点とした
        累積位置であり、従来コードのように区間変位を
        後から複雑に加算する必要はない。
        """

        curvature_array = self._normalize_curvatures(
            curvatures
        )

        state = DynamicState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_velocity=float(
                self.ego_current_velocity.linear
            ),
            yaw_rate=float(
                self.ego_current_velocity.angular
            ),
        )

        ego_positions = np.zeros(
            (
                self.horizon_length,
                2,
            ),
            dtype=float,
        )

        predicted_states = []

        for horizon_idx, (
            curvature,
            horizon_duration,
        ) in enumerate(zip(
            curvature_array,
            self.horizon_durations,
        )):

            state = self._simulate_interval(
                state,
                float(curvature),
                float(horizon_duration),
            )

            ego_positions[horizon_idx] = [
                state.x,
                state.y,
            ]

            predicted_states.append(
                DynamicState(**vars(state))
            )

        self.last_predicted_states = (
            predicted_states
        )

        self.last_predicted_yaws = np.array(
            [
                state.yaw
                for state in predicted_states
            ],
            dtype=float,
        )

        return ego_positions

    def predict_pose(
        self,
        curvature: float,
        horizon_time: float,
    ) -> Pose:
        """
        現在の実測V, omegaから開始し、
        単一曲率をhorizon_timeだけ維持したときの
       相対Poseを返す。

        extremum_seeking_mpc.pyでは、
        最初の区間の予測ヨー角からヨー角速度指令を
        生成するために使用される。

        従来モデル:
            yaw = V * kappa * T

        本モデル:
            電流・トルク・ヨー慣性を考慮して
            omega(t)を積分し、yawを求める。
        """

        initial_state = DynamicState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_velocity=float(
                self.ego_current_velocity.linear
            ),
            yaw_rate=float(
                self.ego_current_velocity.angular
            ),
        )

        predicted_state = self._simulate_interval(
            initial_state,
            float(curvature),
            float(horizon_time),
        )

        return Pose(
            pos=Position2d(
                predicted_state.x,
                predicted_state.y,
            ),
            yaw=predicted_state.yaw,
        )

    @staticmethod
    def create_rotation_matrix(
        angle: float,
    ) -> np.ndarray:
        """二次元回転行列を生成する。"""

        return np.array(
            [
                [
                    np.cos(angle),
                    -np.sin(angle),
                ],
                [
                    np.sin(angle),
                    np.cos(angle),
                ],
            ],
            dtype=float,
        )

    @staticmethod
    def rotate_position(
        position: Position2d,
        angle: float,
    ) -> np.ndarray:
        """
        Position2dを指定角度だけ回転する。

        既存コードとの互換性のため残している。
        """

        rotation_matrix = (
            PosePredictor.create_rotation_matrix(
                angle
            )
        )

        return (
            rotation_matrix
            @ position.as_array()
        )

    def predict_relative_seek_positions(
        self,
        ego_positions: np.ndarray,
    ) -> np.ndarray:
        """
        各ホライズンでのseek pointの横方向オフセットを、
        予測された車体姿勢に合わせて回転する。

        戻り値:
            shape =
                (
                    horizon_length,
                    2,
                    num_seek_points
                )

        ここでは車体中心位置は加えず、
        回転済みオフセットだけを返す。
        """

        del ego_positions
        # public interface互換のため引数は残すが、
        # オフセット計算には位置そのものを使わない。

        num_seek_points = (
            self.seek_y_positions.shape[1]
        )

        relative_seek_positions = np.zeros(
            (
                self.horizon_length,
                2,
                num_seek_points,
            ),
            dtype=float,
        )

        for horizon_idx in range(
            self.horizon_length
        ):
            lateral_offsets = np.asarray(
                self.seek_y_positions[horizon_idx],
                dtype=float,
            )

            local_offsets = np.vstack(
                [
                    np.zeros_like(
                        lateral_offsets
                    ),
                    lateral_offsets,
                ]
            )

            rotation_matrix = (
                self.create_rotation_matrix(
                    self.last_predicted_yaws[
                        horizon_idx
                    ]
                )
            )

            relative_seek_positions[
                horizon_idx
            ] = (
                rotation_matrix
                @ local_offsets
            )

        return relative_seek_positions

    def predict_absolute_seek_positions(
        self,
        ego_positions: np.ndarray,
        relative_seek_positions: np.ndarray,
    ) -> np.ndarray:
        """
        車体中心の累積予測位置へ、
        車体姿勢に応じて回転した横方向seek offsetを加える。

        戻り値は既存RiskCalculatorが期待する、

            shape =
                (
                    horizon_length,
                    2,
                    num_seek_points
                )

        である。
        """

        ego_positions_array = np.asarray(
            ego_positions,
            dtype=float,
        )

        relative_array = np.asarray(
            relative_seek_positions,
            dtype=float,
        )

        expected_position_shape = (
            self.horizon_length,
            2,
        )

        if (
            ego_positions_array.shape
            != expected_position_shape
        ):
            raise ValueError(
                "ego_positions must have shape "
                f"{expected_position_shape}, "
                f"but got {ego_positions_array.shape}."
            )

        if (
            relative_array.ndim != 3
            or relative_array.shape[0]
            != self.horizon_length
            or relative_array.shape[1] != 2
        ):
            raise ValueError(
                "relative_seek_positions must have shape "
                "(horizon_length, 2, num_seek_points)."
            )

        absolute_seek_positions = (
            ego_positions_array[:, :, np.newaxis]
            + relative_array
        )

        return absolute_seek_positions
