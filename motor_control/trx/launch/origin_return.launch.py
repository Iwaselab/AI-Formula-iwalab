from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trx',
            executable='test_script',
            name='motor_control',
            output='screen'
        )
    ])