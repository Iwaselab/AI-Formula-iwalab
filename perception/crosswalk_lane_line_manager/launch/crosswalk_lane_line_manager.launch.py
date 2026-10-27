import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    package_dir = get_package_share_directory("crosswalk_lane_line_manager")
    config_path = osp.join(package_dir, "config", "crosswalk_lane_line_manager.yaml")
    frame_ids, topic_names = get_frame_ids_and_topic_names()

    node = Node(
        package="crosswalk_lane_line_manager",
        executable="crosswalk_lane_line_manager",
        name="crosswalk_lane_line_manager",
        namespace="/aiformula_perception",
        output="screen",
        emulate_tty=True,
        parameters=[
            config_path,
            {
                "odom_frame_id": frame_ids["odom"],
                "vehicle_frame_id": frame_ids["base_footprint"],
            },
        ],
        remappings=[
            ("camera_lane_lines/left", topic_names["perception"]["camera_lane_lines"]["left"]),
            ("camera_lane_lines/right", topic_names["perception"]["camera_lane_lines"]["right"]),
            ("camera_lane_lines/center", topic_names["perception"]["camera_lane_lines"]["center"]),
            ("crosswalk_detection", topic_names["perception"]["objects"]["bounding_box"]),
            ("odometry", topic_names["sensing"]["odometry"]["gyro"]),
            ("lane_lines/left", topic_names["perception"]["lane_lines"]["left"]),
            ("lane_lines/right", topic_names["perception"]["lane_lines"]["right"]),
            ("lane_lines/center", topic_names["perception"]["lane_lines"]["center"]),
        ],
    )
    return LaunchDescription([node])
