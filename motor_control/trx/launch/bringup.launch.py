from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    serial_port = LaunchConfiguration('serial_port').perform(context)
    can_id1 = int(LaunchConfiguration('can_id1').perform(context))
    can_id2 = int(LaunchConfiguration('can_id2').perform(context))
    controller_rate = float(LaunchConfiguration('controller_rate').perform(context))
    model = os.path.join(
        get_package_share_directory('trx'),
        'urdf', 'trx_s.urdf'
    )
    rviz_config = os.path.join(
        get_package_share_directory('trx'),
        'trx.rviz'
    )

    with open(model, 'r') as infp:
        robot_desc = infp.read()

    return [
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('trx'),
                    'config',
                    'hand_controllers.yaml'
                )
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_trajectory_controller',
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='trx_s'),
        DeclareLaunchArgument('can_id1', default_value='1'),
        DeclareLaunchArgument('can_id2', default_value='2'),
        DeclareLaunchArgument('controller_rate', default_value='20'),

        OpaqueFunction(function=launch_setup)
    ])
