import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = 'detection_fusion'
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()
    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory(PACKAGE_NAME), 'config', 'detection_fusion.yaml'),
    )

    fusion_node = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace='/aiformula_perception',
        output='screen',
        parameters=[[*ROS_PARAM_CONFIG]],
        remappings=[
            ("sub_image",
             TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"]),
            ('sub_lane_new',
             TOPIC_NAMES['perception']['mask_image_new']),
            ('sub_lane_old',
             TOPIC_NAMES['perception']['mask_image_old']),
            ('pub_lane_mask',
             TOPIC_NAMES['perception']['mask_image']),
            ('pub_annotated_image',
             TOPIC_NAMES['visualization']['annotated_image']),
        ],
    )

    return LaunchDescription([
        fusion_node
    ])
