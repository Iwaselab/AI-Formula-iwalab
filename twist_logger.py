#!/usr/bin/env python3
"""
pub_twist_command (geometry_msgs/Twist) を購読し、
受信時刻付きでコンソール表示 & CSV保存するロガーノード。

使い方:
    python3 twist_logger.py --topic /aiformula_planning/<remap先トピック名> --output twist_log.csv

Twist型自体にはheader.stampが無いため、ここでは
「ノードがメッセージを受信した時刻(self.get_clock().now())」を
タイムスタンプとして付与している。厳密なパブリッシュ時刻ではなく
受信時刻である点に注意(ネットワーク/DDS遅延を含む)。
"""

import argparse
import csv
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistLogger(Node):
    def __init__(self, topic: str, output_path: str, buffer_size: int = 10):
        super().__init__('twist_logger')

        self.output_path = output_path
        self.csv_file = open(self.output_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['stamp_sec', 'stamp_nanosec', 'linear_x', 'angular_z']
        )

        self.subscription = self.create_subscription(
            Twist, topic, self.twist_callback, buffer_size
        )

        self.get_logger().info(f"Subscribing to '{topic}'")
        self.get_logger().info(f"Logging to '{self.output_path}'")

    def twist_callback(self, msg: Twist) -> None:
        now = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"[{now.sec}.{now.nanosec:09d}] "
            f"linear.x={msg.linear.x:.6f}  angular.z={msg.angular.z:.6f}"
        )

        self.csv_writer.writerow(
            [now.sec, now.nanosec, msg.linear.x, msg.angular.z]
        )
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Twist topic logger with timestamps')
    parser.add_argument(
        '--topic', type=str, default='/aiformula_planning/pub_twist_command',
        help='購読するTwistトピック名(remap後の実際のトピック名を指定)'
    )
    parser.add_argument(
        '--output', type=str, default='twist_log.csv',
        help='出力CSVファイルパス'
    )
    parsed_args, remaining = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining)
    node = TwistLogger(parsed_args.topic, parsed_args.output)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
