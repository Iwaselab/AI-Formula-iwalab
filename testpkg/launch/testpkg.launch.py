from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    testnode = Node(
        package='testpkg',
        executable='testnode',
        name='test_node',
        remappings= [('test_topic_pub', 'test_topic_string')]
    ) 
    testnode2 = Node(
        package='testpkg',
        executable='testnode2',
        name='test_node2',
        remappings= [('test_topic_sub', 'test_topic_string')]
    )

    return LaunchDescription([testnode, testnode2])