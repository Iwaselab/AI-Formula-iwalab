#!/usr/bin/env python3
"""
extremum_seeking_mpc_torque_model.py

リスクベースのExtremum Seekingによる曲率探索と、
トルク・加速度を考慮したPosePredictorを接続する上位制御ノード。

本ノードの基本思想
------------------
本システムでは、経路を直接幾何学的に決めるのではなく、

    1. 現在の曲率列を車両が実行したときの将来軌道を予測する
    2. その予測軌道上で道路・障害物リスクを評価する
    3. PathOptimizerがリスクを小さくする方向へ曲率列を更新する
    4. 更新後の先頭曲率から速度・ヨー角速度目標値を生成する

という閉ループを構成する。

今回の重要な変更
----------------
PosePredictorが、

    曲率
      → 左右電流
      → 車輪トルク
      → 並進加速度・ヨー角加速度
      → V, omega
      → X, Y, yaw

を予測する動的モデルへ変更された。

したがって、同じ曲率候補であっても、

    ・車体質量
    ・ヨー慣性モーメント
    ・モータ電流上限
    ・電流-トルク係数
    ・下位速度制御系の応答時間

によって予測軌道が変わる。

この予測軌道からリスクを計算するため、
トルク・加速度モデルの影響が最終的に
PathOptimizerの曲率更新へ反映される。

外部インターフェース
--------------------
出力:
    pub_twist_command : geometry_msgs/msg/Twist

        linear.x  = 並進速度目標値 V_ref [m/s]
        angular.z = ヨー角速度目標値 omega_ref [rad/s]

Twistを加速度指令へ変更しない理由
--------------------------------
下位のmotor_controllerは、

    V_ref - V
    omega_ref - omega

から符号付きの左右電流指令を生成する。

したがって、前進中であってもV_refを現在速度より小さくすれば、
motor_controllerは負の電流を生成し、能動的な制動を行う。

上位ノードは従来どおり速度・ヨー角速度の目標値を出し、
実際の加速・制動と旋回立ち上がりは、
下位電流制御系と車両ダイナミクスによって生じる。

PosePredictorは、その動的応答を予測してリスク評価へ戻す。
"""

from typing import Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from common_python.get_ros_parameter import get_ros_parameter
from .object_risk_calculator import ObjectRiskCalculator
from .path_optimizer import PathOptimizer
from .pose_predictor_torque_model import PosePredictor
from .road_risk_calculator import RoadRiskCalculator
from .util import Side, Vector2


class ExtremumSeekingMpcTorqueModel(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc_torque_model')

        self.init_parameters()
        self.init_members()
        self.init_connections()

        # 予測軌道可視化用TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # PathOptimizerが扱う曲率列。
        #
        # この値は「最適化変数としての曲率」であり、
        # 実際にPosePredictorおよび車両指令へ使用するときは
        # curvature_gainを掛けた実効曲率へ変換する。
        self.curvatures = np.zeros(
            self.horizon_length,
            dtype=float,
        )

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

    # ===============================
    # ROSパラメータ
    # ===============================

    def init_parameters(self) -> None:

        planned_speed_parameter = get_ros_parameter(
            self,
            "planned_speed",
        )
        self.ego_target_velocity = self._first_scalar(
            planned_speed_parameter,
            "planned_speed",
        )

        self.predict_horizon = np.asarray(
            get_ros_parameter(
                self,
                "horizon_times",
            ),
            dtype=float,
        ).reshape(-1)

        if self.predict_horizon.size == 0:
            raise ValueError(
                "horizon_times must contain at least one value."
            )

        if np.any(self.predict_horizon <= 0.0):
            raise ValueError(
                "All horizon_times must be positive."
            )

        if np.any(np.diff(self.predict_horizon) <= 0.0):
            raise ValueError(
                "horizon_times must be strictly increasing."
            )

        self.horizon_length = len(
            self.predict_horizon
        )

        self.curvature_gain = float(
            get_ros_parameter(
                self,
                "curvature_gain",
            )
        )

        self.benefit_gain = float(
            get_ros_parameter(
                self,
                "benefit_gain",
            )
        )

        self.deceleration_angle_maximum = float(
            get_ros_parameter(
                self,
                "velocity_control."
                "deceleration_angle_maximum",
            )
        )

        self.deceleration_gain = float(
            get_ros_parameter(
                self,
                "velocity_control.deceleration_gain",
            )
        )

        self.base_footprint_frame_id = str(
            get_ros_parameter(
                self,
                "base_footprint_frame_id",
            )
        )

        self.control_period = float(
            get_ros_parameter(
                self,
                "control_period",
            )
        )

        self.buffer_size = int(
            get_ros_parameter(
                self,
                "buffer_size",
            )
        )

        if self.control_period <= 0.0:
            raise ValueError(
                "control_period must be positive."
            )

        if self.buffer_size <= 0:
            raise ValueError(
                "buffer_size must be positive."
            )

        if self.deceleration_angle_maximum < 0.0:
            raise ValueError(
                "velocity_control."
                "deceleration_angle_maximum "
                "must be non-negative."
            )

        if self.deceleration_gain < 0.0:
            raise ValueError(
                "velocity_control.deceleration_gain "
                "must be non-negative."
            )

    @staticmethod
    def _first_scalar(
        value,
        parameter_name: str,
    ) -> float:
        """
        planned_speedがスカラーまたは配列のどちらであっても、
        最初の値をfloatとして取得する。

        従来コードではplanned_speed[0]を使用していたため、
        既存YAMLが配列形式でも互換性を保つ。
        """

        array = np.asarray(
            value,
            dtype=float,
        ).reshape(-1)

        if array.size == 0:
            raise ValueError(
                f"{parameter_name} must not be empty."
            )

        return float(array[0])

    # ===============================
    # メンバ初期化
    # ===============================

    def init_members(self) -> None:

        # リスク計算器のアルゴリズムは変更しない。
        # PosePredictorが返す将来seek pointの位置を評価するだけなので、
        # 車両が速度制御型か電流制御型かには直接依存しない。
        self.object_risk_calculator = (
            ObjectRiskCalculator(
                self,
                self.buffer_size,
            )
        )

        self.road_risk_calculator = (
            RoadRiskCalculator(
                self,
                self.buffer_size,
            )
        )

        # PathOptimizerも変更しない。
        # リスクを小さくする曲率列をExtremum Seekingで更新する。
        self.path_optimizer = PathOptimizer(
            self,
            self.control_period,
        )

        init_seek_points_array = np.array(
            [
                controller.seek_points
                for controller
                in self.path_optimizer
                .extremum_seeking_controllers
            ],
            dtype=float,
        )

        if (
            len(init_seek_points_array)
            != self.horizon_length
        ):
            raise ValueError(
                "The number of PathOptimizer controllers "
                "must match the number of horizon_times."
            )

        # 修正済みPosePredictorを生成する。
        #
        # PosePredictor内部では、曲率から電流・トルク・加速度を経て
        # 将来の車体位置と姿勢を逐次積分する。
        self.pose_predictor = PosePredictor(
            self,
            self.predict_horizon.tolist(),
            init_seek_points_array,
            self.buffer_size,
        )

    # ===============================
    # ROS接続
    # ===============================

    def init_connections(self) -> None:

        self.twist_pub = self.create_publisher(
            Twist,
            'pub_twist_command',
            self.buffer_size,
        )

    # ===============================
    # 曲率と予測軌道
    # ===============================

    def calculate_effective_curvatures(
        self,
        curvatures: np.ndarray,
    ) -> np.ndarray:
        """
        PathOptimizerの曲率列へcurvature_gainを適用する。

        従来コードでは、
            ・PosePredictorにはgain適用前の曲率
            ・実車指令にはgain適用後のヨー角速度
        を使用していた。

        その場合、リスク評価に用いる予測軌道と、
        実際に車両へ与える旋回指令が一致しない。

        本実装では、PosePredictorと実車指令の両方に
        同じ実効曲率

            kappa_effective =
                curvature_gain * kappa_optimizer

        を使用する。
        """

        curvature_array = np.asarray(
            curvatures,
            dtype=float,
        ).reshape(-1)

        if curvature_array.size != self.horizon_length:
            raise ValueError(
                "curvatures length must match "
                "the prediction horizon length."
            )

        return (
            self.curvature_gain
            * curvature_array
        )

    def predict_ego_position(
        self,
        effective_curvatures: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        実効曲率列を動的PosePredictorへ与え、
        車体中心位置とリスク評価用seek pointを予測する。
        """

        ego_positions = (
            self.pose_predictor
            .predict_relative_ego_positions(
                effective_curvatures
            )
        )

        seek_positions_relative = (
            self.pose_predictor
            .predict_relative_seek_positions(
                ego_positions
            )
        )

        seek_positions = (
            self.pose_predictor
            .predict_absolute_seek_positions(
                ego_positions,
                seek_positions_relative,
            )
        )

        return ego_positions, seek_positions

    # ===============================
    # リスク計算
    # ===============================

    def calculate_object_risk(
        self,
        seek_positions: np.ndarray,
    ) -> np.ndarray:

        return (
            self.object_risk_calculator
            .compute_object_risk(
                seek_positions
            )
        )

    def calculate_road_risk(
        self,
        seek_positions: np.ndarray,
    ) -> Tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:

        left_road_risk, left_y_hat = (
            self.road_risk_calculator
            .compute_road_risk(
                seek_positions,
                Side.LEFT,
            )
        )

        right_road_risk, right_y_hat = (
            self.road_risk_calculator
            .compute_road_risk(
                seek_positions,
                Side.RIGHT,
            )
        )

        benefit = (
            self.road_risk_calculator
            .get_benefit_value(
                seek_positions,
                left_y_hat,
                right_y_hat,
            )
        )

        return (
            left_road_risk,
            right_road_risk,
            benefit,
        )

    def calculate_total_risk(
        self,
        object_risk: np.ndarray,
        left_road_risk: np.ndarray,
        right_road_risk: np.ndarray,
        benefit: np.ndarray,
    ) -> np.ndarray:
        """
        障害物リスク、左右道路境界リスク、
        道路中央を走るbenefitから総評価値を生成する。

        この評価式自体は従来の設計思想を維持する。
        変更されたのは、評価対象となるseek_positionsが
        トルク・加速度制約を含む予測軌道上に置かれる点である。
        """

        return (
            np.asarray(
                object_risk,
                dtype=float,
            )
            + np.asarray(
                left_road_risk,
                dtype=float,
            )
            + np.asarray(
                right_road_risk,
                dtype=float,
            )
            - self.benefit_gain
            * np.asarray(
                benefit,
                dtype=float,
            )
        )

    # ===============================
    # 曲率更新
    # ===============================

    def update_curvatures(
        self,
        total_risk: np.ndarray,
    ) -> np.ndarray:
        """
        総リスクをPathOptimizerへ渡し、
        次の曲率列を取得する。

        旧コードでは、新しいcurvaturesを計算した直後に、
        古いself.curvatures[0]を使ってyawを計算する
        1制御周期のずれがあった。

        本実装では、返された新しい曲率列を直ちに保存し、
        同じ曲率列から今回の速度・ヨー角速度指令を生成する。
        """

        updated_curvatures = np.asarray(
            self.path_optimizer
            .apply_extremum_seeking_control(
                total_risk
            ),
            dtype=float,
        ).reshape(-1)

        if (
            updated_curvatures.size
            != self.horizon_length
        ):
            raise ValueError(
                "PathOptimizer returned a curvature "
                "sequence whose length does not match "
                "horizon_times."
            )

        if not np.all(
            np.isfinite(updated_curvatures)
        ):
            raise ValueError(
                "PathOptimizer returned NaN or Inf "
                "in the curvature sequence."
            )

        self.curvatures = updated_curvatures

        return self.curvatures.copy()

    # ===============================
    # 曲率から速度・ヨー角速度目標値を生成
    # ===============================

    def calculate_linear_velocity_reference(
        self,
        effective_curvature: float,
    ) -> float:
        """
        先頭予測区間で予想される旋回角が大きい場合、
        目標速度を低下させる。

        まず公称目標速度V_nominalを用いて、

            Delta_yaw_est
                = |V_nominal
                   * kappa_effective
                   * T_0|

        を計算する。

        閾値を超える場合、

            |V_ref|
                = max(
                    |V_nominal|
                    - gain
                      * (Delta_yaw_est
                         - threshold),
                    0
                  )

        とする。

        速度が正の状態でV_refを小さくすると、
        下位motor_controllerの速度偏差が負となり、
        負の電流指令、すなわち制動トルクが生成される。
        """

        first_horizon_time = float(
            self.predict_horizon[0]
        )

        estimated_yaw_angle = abs(
            self.ego_target_velocity
            * effective_curvature
            * first_horizon_time
        )

        excess_yaw_angle = (
            estimated_yaw_angle
            - self.deceleration_angle_maximum
        )

        if excess_yaw_angle <= 0.0:
            return self.ego_target_velocity

        target_speed_magnitude = max(
            abs(self.ego_target_velocity)
            - self.deceleration_gain
            * excess_yaw_angle,
            0.0,
        )

        target_direction = (
            1.0
            if self.ego_target_velocity >= 0.0
            else -1.0
        )

        return (
            target_direction
            * target_speed_magnitude
        )

    @staticmethod
    def calculate_yaw_rate_reference(
        linear_velocity_reference: float,
        effective_curvature: float,
    ) -> float:
        """
        目標曲率と速度目標値からヨー角速度目標値を求める。

            omega_ref =
                V_ref * kappa_effective

        ここで生成するのは「目標ヨー角速度」であり、
        PosePredictorが計算した実現済み平均ヨー角速度ではない。

        動的予測値をそのまま指令に使うと、
        ヨー応答の遅れによって小さくなった予測omegaを
        再び目標値として与えることになり、
        必要な旋回指令まで弱めてしまう。

        車両の応答遅れは指令値を弱めるのではなく、
        PosePredictor内で実軌道の遅れとして表現する。
        """

        return float(
            linear_velocity_reference
            * effective_curvature
        )

    def calculate_control_reference(
        self,
        effective_curvatures: np.ndarray,
    ) -> Tuple[float, float]:
        """
        更新後曲率列の先頭値から、
        今回送信するV_refとomega_refを計算する。
        """

        first_curvature = float(
            effective_curvatures[0]
        )

        linear_velocity_reference = (
            self.calculate_linear_velocity_reference(
                first_curvature
            )
        )

        yaw_rate_reference = (
            self.calculate_yaw_rate_reference(
                linear_velocity_reference,
                first_curvature,
            )
        )

        return (
            linear_velocity_reference,
            yaw_rate_reference,
        )

    # ===============================
    # Twist出力
    # ===============================

    def publish_cmd_vel(
        self,
        vehicle_linear_velocity: float,
        yaw_rate: float,
    ) -> None:
        """
        下位motor_controllerへ速度・ヨー角速度目標値を送信する。

        motor_controller側で速度・ヨー角速度偏差を
        左右の符号付き電流へ変換するため、
        ここでは加速度や電流を直接送らない。
        """

        twist_msg = Twist()
        twist_msg.linear.x = float(
            vehicle_linear_velocity
        )
        twist_msg.angular.z = float(
            yaw_rate
        )

        self.twist_pub.publish(twist_msg)

    # ===============================
    # 予測軌道TF表示
    # ===============================

    def tf_viewer(
        self,
        ego_positions: np.ndarray,
    ) -> None:
        """
        更新後の曲率列から予測された各ホライズン位置をTF表示する。

        修正済みPosePredictorが返すego_positionsは、
        すべて予測開始時base_footprintを原点とした累積位置である。

        そのため、従来のように

            base -> point0 -> point1 -> point2

        と連結すると位置を二重に加算してしまう。

        本実装ではすべての予測点を、

            base_footprint -> predicted_ego_pose_i

        として直接配信する。
        """

        positions = np.asarray(
            ego_positions,
            dtype=float,
        )

        predicted_yaws = np.asarray(
            self.pose_predictor
            .last_predicted_yaws,
            dtype=float,
        )

        now = self.get_clock().now().to_msg()

        for horizon_idx, position in enumerate(
            positions
        ):
            yaw = (
                float(predicted_yaws[horizon_idx])
                if horizon_idx
                < predicted_yaws.size
                else 0.0
            )

            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = (
                self.base_footprint_frame_id
            )
            transform.child_frame_id = (
                f"predicted_ego_pose_{horizon_idx}"
            )

            transform.transform.translation.x = float(
                position[Vector2.X]
            )
            transform.transform.translation.y = float(
                position[Vector2.Y]
            )
            transform.transform.translation.z = 0.0

            # ヨー角のみを持つQuaternion
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = float(
                np.sin(0.5 * yaw)
            )
            transform.transform.rotation.w = float(
                np.cos(0.5 * yaw)
            )

            self.tf_broadcaster.sendTransform(
                transform
            )

    # ===============================
    # 制御周期処理
    # ===============================

    def publish_cmd_vel_timer_callback(
        self,
    ) -> None:
        """
        1制御周期の処理順序。

        A. 現在の曲率列を、curvature_gainを含む実効曲率へ変換
        B. 動的PosePredictorで将来軌道とseek pointを予測
        C. 障害物・道路リスクを計算
        D. PathOptimizerで曲率列を更新
        E. 更新後曲率からV_ref, omega_refを生成
        F. 更新後曲率の予測軌道をTF表示
        G. Twistを下位motor_controllerへ送信

        曲率更新に使われるリスクは、
        トルク・加速度制約を含む予測軌道から計算される。
        """

        # Odometry受信前は、現在速度・ヨー角速度を確認できないため、
        # ゼロ指令を維持する。
        if not self.pose_predictor.has_received_odometry:

            if not self.waiting_for_odometry_logged:
                self.get_logger().warning(
                    "Waiting for odometry. "
                    "Publishing zero command."
                )
                self.waiting_for_odometry_logged = True

            self.publish_cmd_vel(
                0.0,
                0.0,
            )
            return

        self.waiting_for_odometry_logged = False

        try:
            # ---------------------------------------
            # 1. 現在曲率列の動的予測
            # ---------------------------------------

            current_effective_curvatures = (
                self.calculate_effective_curvatures(
                    self.curvatures
                )
            )

            (
                _evaluated_ego_positions,
                evaluated_seek_positions,
            ) = self.predict_ego_position(
                current_effective_curvatures
            )

            # ---------------------------------------
            # 2. 予測軌道上のリスク評価
            # ---------------------------------------

            object_risk = (
                self.calculate_object_risk(
                    evaluated_seek_positions
                )
            )

            (
                left_road_risk,
                right_road_risk,
                benefit,
            ) = self.calculate_road_risk(
                evaluated_seek_positions
            )

            total_risk = (
                self.calculate_total_risk(
                    object_risk,
                    left_road_risk,
                    right_road_risk,
                    benefit,
                )
            )

            # ---------------------------------------
            # 3. 曲率列の更新
            # ---------------------------------------

            updated_curvatures = (
                self.update_curvatures(
                    total_risk
                )
            )

            updated_effective_curvatures = (
                self.calculate_effective_curvatures(
                    updated_curvatures
                )
            )

            # ---------------------------------------
            # 4. 更新後曲率から制御目標値を生成
            # ---------------------------------------

            (
                vehicle_linear_velocity,
                yaw_rate,
            ) = self.calculate_control_reference(
                updated_effective_curvatures
            )

            # ---------------------------------------
            # 5. 更新後曲率の軌道を再予測
            # ---------------------------------------
            #
            # リスク評価に用いた軌道は更新前曲率の軌道である。
            # TFには、今回実際に指令へ使用する更新後曲率の
            # 予測軌道を表示する。

            (
                commanded_ego_positions,
                _commanded_seek_positions,
            ) = self.predict_ego_position(
                updated_effective_curvatures
            )

            # ---------------------------------------
            # 6. 指令送信と可視化
            # ---------------------------------------

            self.publish_cmd_vel(
                vehicle_linear_velocity,
                yaw_rate,
            )

            self.tf_viewer(
                commanded_ego_positions
            )

        except Exception as error:
            # 制御計算に異常が生じた場合、
            # 前回指令を保持せずゼロ指令へ移行する。
            self.get_logger().error(
                "Extremum Seeking control cycle failed: "
                f"{error}"
            )

            self.publish_cmd_vel(
                0.0,
                0.0,
            )

    # ===============================
    # 停止処理
    # ===============================

    def stop_vehicle(self) -> None:
        """
        上位目標値をゼロにする。

        走行中にV_ref=0を送ると、
        下位motor_controllerではV_ref - Vが負となり、
        速度が残っている間は負電流による制動が働く。
        """

        self.publish_cmd_vel(
            0.0,
            0.0,
        )


# ===============================
# メイン関数
# ===============================

def main(args=None) -> None:

    rclpy.init(args=args)
    extremum_seeking_mpc = ExtremumSeekingMpcTorqueModel()

    try:
        rclpy.spin(extremum_seeking_mpc)

    except KeyboardInterrupt:
        extremum_seeking_mpc.get_logger().info(
            "KeyboardInterrupt received. "
            "Publishing stop command."
        )

    finally:
        extremum_seeking_mpc.stop_vehicle()
        extremum_seeking_mpc.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
