import os
from typing import Tuple
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from aiformula_interfaces.msg import RectMultiArray, Rect
from common_python.get_ros_parameter import get_ros_parameter


class YoloObjectDetector(Node):

    def __init__(self):
        super().__init__('yolo_object_detector')
        device, path_to_weights = self.get_params()
        self.cv_bridge = CvBridge()
        self.confidence_threshold = float(get_ros_parameter(self, 'confidence_threshold'))
        self.iou_threshold = float(get_ros_parameter(self, 'iou_threshold'))
        self.init_model(device, path_to_weights)

        buffer_size = 10
        self.image_sub = self.create_subscription(Image, 'sub_image', self.image_callback, buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_bbox', buffer_size)
        self.annotated_image_pub = self.create_publisher(Image, 'pub_annotated_image', buffer_size)

    def get_params(self) -> Tuple[str, str]:
        device = str(get_ros_parameter(self, 'use_device'))
        path_to_weights = str(get_ros_parameter(self, 'weight_path'))
        return device, path_to_weights

    def init_model(self, device: str, path_to_weights: str) -> None:
        if not os.path.exists(path_to_weights):
            raise FileNotFoundError(f'Weight file not found: {path_to_weights}')
        self.get_logger().info(f'Using weights: {path_to_weights}')
        self.model = YOLO(path_to_weights)
        self.device = device

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warning(f'CvBridgeError occurred: {str(e)}')
            return

        results = self.model.predict(
            source=cv_image,
            device=self.device,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            verbose=False,
        )[0]

        bbox_msg = RectMultiArray()
        bbox_msg.header = msg.header

        boxes = results.boxes
        if boxes is None:
            self.rects_pub.publish(bbox_msg)
            return

        cv_image_copy = cv_image.copy()

        for box in boxes.xyxy.tolist():
            x1, y1, x2, y2 = [int(v) for v in box]
            cv2.rectangle(cv_image_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)

            rect = Rect()
            rect.x = float(x1)
            rect.y = float(y1)
            rect.width = float(x2 - x1)
            rect.height = float(y2 - y1)
            bbox_msg.rects.append(rect)

        self.rects_pub.publish(bbox_msg)

        annotated_msg = self.cv_bridge.cv2_to_imgmsg(cv_image_copy, 'bgr8')
        annotated_msg.header = msg.header
        self.annotated_image_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()