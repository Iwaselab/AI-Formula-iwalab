# OSのパス操作を行うためのモジュール
import os

# ROS 2パッケージのインストール先を取得するためのモジュール
from ament_index_python.packages import get_package_share_directory

# ROS 2 launchのLaunchDescription
from launch import LaunchDescription

# launch引数を宣言するためのクラス
from launch.actions import DeclareLaunchArgument

# launch時に指定された引数を取得するためのクラス
from launch.substitutions import LaunchConfiguration

# ROS 2ノードを起動するためのクラス
from launch_ros.actions import Node


def generate_launch_description():

    # このパッケージの名前を指定する
    package_name = "control_test"

    # control_testパッケージのインストール先を取得する
    package_share_directory = get_package_share_directory(
        package_name
    )

    # CSVファイルのパスをlaunch引数として指定できるようにする
    csv_file_arg = DeclareLaunchArgument(
        "csv_file",

        # デフォルトで使用するCSVファイル
        default_value=os.path.join(
            package_share_directory,
            "data",
            "controller_test01.csv"
        ),

        # launch --show-argsなどで表示される説明
        description="Path to the CSV file used for controller testing"
    )

    # controller_testノードを起動する
    controller_test_node = Node(

        # ROS 2パッケージ名
        package=package_name,

        # setup.pyのconsole_scriptsで登録する実行ファイル名
        executable="controller_test",

        # ROS 2上でのノード名
        name="controller_test",

        # ノードの出力をターミナルに表示する
        output="screen",

        # controller_test.pyへCSVファイルのパスを渡す
        parameters=[
            {
                "csv_file": LaunchConfiguration("csv_file")
            }
        ],
    )

    # LaunchDescriptionを作成して返す
    return LaunchDescription([

        # CSVファイルのlaunch引数を登録する
        csv_file_arg,

        # controller_testノードを起動する
        controller_test_node,
    ])