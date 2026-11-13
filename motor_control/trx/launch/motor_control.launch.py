from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trx',
            executable='rear_steering_node',
            name='motor_control',
            output='screen'
        )
    ])