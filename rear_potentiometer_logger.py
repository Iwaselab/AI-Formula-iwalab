#!/usr/bin/env python3
"""
/aiformula_sensing/rear_potentiometer/yaw (std_msgs/Float32) を購読し、
受信時刻付きでコンソール表示 & CSV保存するロガーノード。

使い方:
    python3 rear_potentiometer_logger.py
    python3 rear_potentiometer_logger.py --topic /aiformula_sensing/rear_potentiometer/yaw --output rear_potentiometer_log.csv

Float32型にはheader.stampがないため、ここでは
「ノードがメッセージを受信した時刻(self.get_clock().now())」を
タイムスタンプとして付与する。
"""

import argparse
import csv
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RearPotentiometerLogger(Node):
    def __init__(self, topic: str, output_path: str, buffer_size: int = 10):
        super().__init__('rear_potentiometer_logger')

        self.output_path = output_path
        self.received_count = 0
        self.csv_file = open(self.output_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['stamp_sec', 'stamp_nanosec', 'yaw_rad']
        )

        self.subscription = self.create_subscription(
            Float32, topic, self.yaw_callback, buffer_size
        )
        self.status_timer = self.create_timer(
            5.0, self.check_received_messages
        )

        self.get_logger().info(f"Subscribing to '{topic}'")
        self.get_logger().info(f"Logging to '{self.output_path}'")

    def yaw_callback(self, msg: Float32) -> None:
        self.received_count += 1
        now = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"[{now.sec}.{now.nanosec:09d}] yaw={msg.data:.6f} rad"
        )

        self.csv_writer.writerow([now.sec, now.nanosec, msg.data])
        self.csv_file.flush()

    def check_received_messages(self) -> None:
        if self.received_count == 0:
            self.get_logger().warn(
                "No yaw messages received. Check ROS_DOMAIN_ID, topic name, "
                "and whether the publisher or rosbag playback is active."
            )

    def destroy_node(self):
        self.get_logger().info(
            f"Recorded {self.received_count} yaw messages "
            f"to '{self.output_path}'"
        )
        self.csv_file.close()
        super().destroy_node()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Rear potentiometer yaw topic logger with timestamps'
    )
    parser.add_argument(
        '--topic',
        type=str,
        default='/aiformula_sensing/rear_potentiometer/yaw',
        help='購読するrear potentiometer yawトピック名'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='rear_potentiometer_log.csv',
        help='出力CSVファイルパス'
    )
    parsed_args, remaining = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining)
    node = RearPotentiometerLogger(parsed_args.topic, parsed_args.output)

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
