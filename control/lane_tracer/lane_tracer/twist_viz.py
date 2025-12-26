#!/usr/bin/env python
"""Simple node to visualize Twist as an arrow Marker in RViz.

Subscribes to a Twist topic and publishes a visualization_msgs/Marker of type
ARROW in the given frame (default `base_footprint`). Run alongside the
simulation to visualize the velocity command.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class TwistViz(Node):
    def __init__(self):
        super().__init__('twist_viz')

        self.declare_parameter('twist_topic', '/aiformula_control/extremum_seeking_mpc/cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('scale_factor', 1.0)
        self.declare_parameter('arrow_color_r', 0.0)
        self.declare_parameter('arrow_color_g', 1.0)
        self.declare_parameter('arrow_color_b', 0.0)
        self.declare_parameter('arrow_alpha', 0.8)

        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scale_factor = float(self.get_parameter('scale_factor').get_parameter_value().double_value)

        r = float(self.get_parameter('arrow_color_r').get_parameter_value().double_value)
        g = float(self.get_parameter('arrow_color_g').get_parameter_value().double_value)
        b = float(self.get_parameter('arrow_color_b').get_parameter_value().double_value)
        a = float(self.get_parameter('arrow_alpha').get_parameter_value().double_value)

        self.color = ColorRGBA(r=r, g=g, b=b, a=a)

        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.sub = self.create_subscription(Twist, twist_topic, self.cb_twist, 10)

        self.get_logger().info(f'TwistViz subscribing to [{twist_topic}] publishing Marker in frame [{self.frame_id}]')

    def cb_twist(self, msg: Twist):
        # Use linear.x as arrow length, scale by parameter
        length = float(msg.linear.x) * self.scale_factor
        if length < 0.0:
            length = 0.0

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'cmd_vel'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Place arrow at robot origin
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.15
        # Identity orientation (points along +X)
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        # Arrow scale: x = length, y = shaft diameter, z = head diameter
        m.scale.x = max(0.01, length)
        m.scale.y = 0.06
        m.scale.z = 0.12

        # Use color to show steering (green -> low, red -> high)
        ang = abs(msg.angular.z)
        # blend from green to red by steering magnitude
        t = min(1.0, ang / 2.0)
        m.color.r = self.color.r * t + (1.0 - t) * 0.0
        m.color.g = self.color.g * (1.0 - t) + t * 0.0
        m.color.b = self.color.b
        m.color.a = self.color.a

        # lifetime short so arrow disappears when no messages
        m.lifetime.sec = 1

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TwistViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
