#!/usr/bin/env python3

# CSVファイルを読み込むためのモジュール
import csv

# ファイルやディレクトリのパスを扱うためのモジュール
import os

# ROS 2 Pythonクライアントライブラリ
import rclpy

# ROS 2のNodeクラス
from rclpy.node import Node

# geometry_msgs/msg/Twistを使用するためのモジュール
from geometry_msgs.msg import Twist


class ControllerTest(Node):

    # ControllerTestノードの初期化
    def __init__(self):

        # ROS 2上でのノード名をcontroller_testにする
        super().__init__("controller_test")

        # CSVファイルのパスをROSパラメータとして宣言する
        self.declare_parameter("csv_file", "")

        # launchファイルから渡されたCSVファイルのパスを取得する
        csv_file = self.get_parameter("csv_file").value

        # CSVファイルのパスが指定されていない場合
        if not csv_file:

            # エラーメッセージを表示する
            self.get_logger().error(
                "CSV file path has not been provided."
            )

            # ノードを終了する
            raise RuntimeError("CSV file is required.")

        # CSVデータを格納するリスト
        self.data = []

        # CSVファイルを読み込む
        self.load_csv(csv_file)

        # 現在再生しているCSVデータのインデックス
        self.current_index = 0

        # 現在有効なlinear.x
        self.current_linear_x = 0.0

        # 現在有効なangular.z
        self.current_angular_z = 0.0

        # CSVの再生開始時刻
        self.start_time = self.get_clock().now()

        # TwistをpublishするPublisherを作成する
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/aiformula_control/controller_test/cmd_vel",
            10
        )

        # 0.01秒 = 100 Hzでpublishするタイマーを作成する
        self.timer = self.create_timer(
            0.01,
            self.publish_command
        )

        # 読み込んだデータ数を表示する
        self.get_logger().info(
            f"Loaded {len(self.data)} data points."
        )

        # 使用するCSVファイルを表示する
        self.get_logger().info(
            f"CSV file: {csv_file}"
        )

        # テスト開始を表示する
        self.get_logger().info(
            "Controller test started."
        )

    def load_csv(self, csv_file):

        # CSVファイルが存在するか確認する
        if not os.path.isfile(csv_file):

            # ファイルが存在しない場合はエラーを表示する
            self.get_logger().error(
                f"CSV file not found: {csv_file}"
            )

            # エラーを発生させる
            raise FileNotFoundError(csv_file)

        # CSVファイルを読み込む
        with open(csv_file, "r", newline="") as file:

            # CSVを辞書形式として読み込む
            reader = csv.DictReader(file)

            # CSVの各行を順番に読み込む
            for row in reader:

                # 1列目の時刻をfloat型に変換する
                t_ms = float(row["t_ms"])

                # 2列目のlinear.xをfloat型に変換する
                linear_x = float(
                    row["gamepad_v_ref_mps"]
                )

                # 3列目のangular.zをfloat型に変換する
                angular_z = float(
                    row["gamepad_yawrate_ref_rps"]
                )

                # 読み込んだデータを辞書として保存する
                self.data.append(
                    {
                        "t_ms": t_ms,
                        "linear_x": linear_x,
                        "angular_z": angular_z,
                    }
                )

        # 念のため時刻の小さい順に並べる
        self.data.sort(
            key=lambda x: x["t_ms"]
        )

        # CSVデータが空の場合
        if not self.data:

            # エラーメッセージを表示する
            self.get_logger().error(
                "CSV file contains no data."
            )

            # エラーを発生させる
            raise RuntimeError(
                "CSV file contains no data."
            )

        # CSVの最初の時刻を取得する
        first_time = self.data[0]["t_ms"]

        # CSVの最初の時刻を0 msとして扱う
        for row in self.data:

            # 各時刻から最初の時刻を引く
            row["t_ms"] -= first_time

    def publish_command(self):

        # CSVデータが存在しない場合は何もしない
        if not self.data:
            return

        # ノード開始から現在までの経過時間を取得する
        elapsed_time = (
            self.get_clock().now()
            - self.start_time
        )

        # 経過時間を秒に変換する
        elapsed_sec = (
            elapsed_time.nanoseconds / 1e9
        )

        # 経過時間をmsに変換する
        elapsed_ms = elapsed_sec * 1000.0

        # 現在時刻までに到達したCSVデータを処理する
        while (
            self.current_index < len(self.data)
            and
            self.data[self.current_index]["t_ms"]
            <= elapsed_ms
        ):

            # 現在のCSVデータを取得する
            current_data = self.data[
                self.current_index
            ]

            # linear.xを更新する
            self.current_linear_x = (
                current_data["linear_x"]
            )

            # angular.zを更新する
            self.current_angular_z = (
                current_data["angular_z"]
            )

            # 次のCSVデータへ進む
            self.current_index += 1

        # Twistメッセージを作成する
        msg = Twist()

        # CSVの最後まで到達していない場合
        if self.current_index < len(self.data):

            # 現在のlinear.xを設定する
            msg.linear.x = self.current_linear_x

            # 現在のangular.zを設定する
            msg.angular.z = self.current_angular_z

        # CSVの最後まで到達した場合
        else:

            # linear.xを0にする
            msg.linear.x = 0.0

            # angular.zを0にする
            msg.angular.z = 0.0

        # Twistをpublishする
        self.cmd_vel_pub.publish(msg)


def main(args=None):

    # ROS 2を初期化する
    rclpy.init(args=args)

    # ControllerTestノードを作成する
    node = ControllerTest()

    try:

        # ノードを実行し続ける
        rclpy.spin(node)

    except KeyboardInterrupt:

        # Ctrl+Cで停止した場合は何もしない
        pass

    finally:

        # ノードを終了する
        node.destroy_node()

        # ROS 2を終了する
        rclpy.shutdown()


# このPythonファイルを直接実行した場合
if __name__ == "__main__":

    # main関数を実行する
    main()