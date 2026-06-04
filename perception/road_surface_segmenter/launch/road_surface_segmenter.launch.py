import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    PACKAGE_NAME = "road_surface_segmenter"
    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory(PACKAGE_NAME), "config", "road_surface_segmenter.yaml"),
    )
    
    # Get yolopv2 weight file path from source directory
    yolopv2_weight_path = os.path.expanduser(
        "~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2/data/weights/yolopv2.pt" #ここで重みのパスを指定
    )

    launch_args = (
        DeclareLaunchArgument(
            "weight_path",
            default_value=yolopv2_weight_path,
            description="Path to the weight pth file."),
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="cuda device, i.e. 0 or cpu",),
    )

    road_surface_segmenter_node = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_perception",
        output="screen",
        parameters=[
            [*ROS_PARAM_CONFIG],
            # Overriding
            {
                "weight_path": LaunchConfiguration("weight_path"),
                "use_device": LaunchConfiguration("use_device"),
            },
        ],
        remappings=[
            ("sub_image", "/aiformula_sensing/zed_node/left_image/undistorted"),
            ("pub_road_surface_mask", "/aiformula_perception/road_surface_mask"),
            ("pub_annotated_image", "/aiformula_perception/road_surface_annotated_image"),
        ],
    )

    return LaunchDescription([
        *launch_args,
        road_surface_segmenter_node,
    ])
