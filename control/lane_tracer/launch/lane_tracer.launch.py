import os.path as osp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os.path as osp


def generate_launch_description():
    PACKAGE_NAME = "lane_tracer"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    launch_args = (
        DeclareLaunchArgument('log_level', default_value='info', description='Logger level'),
    )

    # Use python -m to run the module directly so this launch works regardless
    # of how the package was installed (console script location differences).
    params_file = osp.join(PACKAGE_DIR, 'config', 'lane_tracer.yaml')
    cmd = [
        'python3', '-m', 'lane_tracer.lane_tracer',
        '--ros-args', '--params-file', params_file,
        '--log-level', LaunchConfiguration('log_level')
    ]

    exec_proc = ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False,
    )

    return LaunchDescription([*launch_args, exec_proc])
