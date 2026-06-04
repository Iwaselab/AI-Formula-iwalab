import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    PACKAGE_NAME = 'yolo_object_detector'
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()
    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory(PACKAGE_NAME), 'config', 'yolo_object_detector.yaml'),
    )

    launch_args = (
        DeclareLaunchArgument(
            'weight_path',
            default_value=osp.join(get_package_share_directory(PACKAGE_NAME), 'weights', 'yolo11n.pt'),
            description='Path to the weight file under package weights/ or an absolute local path'),
        DeclareLaunchArgument('use_device', default_value='0', description='cuda device or cpu'),
    )

    yolo_node = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace='/aiformula_perception',
        output='screen',
        parameters=[[*ROS_PARAM_CONFIG],
                    {
                        'weight_path': LaunchConfiguration('weight_path'),
                        'use_device': LaunchConfiguration('use_device'),
                    }],
        remappings=[
            ('sub_image', TOPIC_NAMES['sensing']['zedx']['left_image']['undistorted']),
            ('pub_bbox', TOPIC_NAMES['perception']['objects']['bounding_box']),
            ('pub_annotated_image', TOPIC_NAMES['visualization']['annotated_image_yolo']),
        ],
    )

    return LaunchDescription([
        *launch_args,
        yolo_node,
    ])
