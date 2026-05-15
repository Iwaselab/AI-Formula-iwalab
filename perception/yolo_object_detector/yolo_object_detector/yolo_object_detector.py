import os
from typing import Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
import glob

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
        path_to_weights = get_ros_parameter(self, 'weight_path')
        return device, path_to_weights

    def init_model(self, device: str, path_to_weights: str) -> None:
        # Resolve and load model. Let exceptions propagate so node startup fails
        # if weights are missing or model loading errors occur.
        if not path_to_weights:
            raise RuntimeError(
                'weight_path parameter is required. Set it in '
                'perception/yolo_object_detector/config/yolo_object_detector.yaml '
                'or pass weight_path:=<path-to-weights> when launching.'
            )

        weight_file = self.resolve_weight_path(path_to_weights)
        self.get_logger().info(f'Using local weights: {weight_file}')
        self.model = YOLO(weight_file)
        self.device = device

    def resolve_weight_path(self, path_to_weights: str) -> str:
        """Resolve weight file path. Supports absolute paths or filenames under weights/ directory."""
        # If absolute path is provided, prefer it; if missing, try basename in source
        if os.path.isabs(path_to_weights):
            if os.path.exists(path_to_weights):
                return path_to_weights
            # convert to basename and fall through to search in share/src
            path_to_weights = os.path.basename(path_to_weights)

        # If relative path (filename only), try to find in share/weights
        try:
            share_dir = get_package_share_directory('yolo_object_detector')
            weights_path = os.path.join(share_dir, 'weights', path_to_weights)
            if os.path.exists(weights_path):
                return weights_path
        except Exception:
            pass
        # Try to find weights in workspace source tree under any 'src/**/perception/yolo_object_detector/weights'
        try:
            cwd = os.getcwd()
            # search upwards for a directory containing 'src'
            for parent in [cwd] + [os.path.join(cwd, *(['..'] * i)) for i in range(1, 8)]:
                parent = os.path.abspath(parent)
                src_dir = os.path.join(parent, 'src')
                if os.path.isdir(src_dir):
                    pattern = os.path.join(src_dir, '**', 'yolo_object_detector', 'weights', path_to_weights)
                    matches = glob.glob(pattern, recursive=True)
                    if matches:
                        return matches[0]
        except Exception:
            pass

        # Otherwise, fail: require local weight file
        raise FileNotFoundError(
            f'Weight file not found: {path_to_weights}. '
            'Place your weight file in perception/yolo_object_detector/weights/ '
            'or pass an absolute path to the file when launching.'
        )

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warning(f'CvBridgeError occurred: {str(e)}')
            return

        bbox_msg = RectMultiArray()
        bbox_msg.header = msg.header

        if self.model is None:
            # no model loaded: publish empty message
            self.rects_pub.publish(bbox_msg)
            return

        results = self.model.predict(
            source=cv_image,
            device=self.device,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            verbose=False,
        )[0]

        boxes = results.boxes
        if boxes is None:
            self.rects_pub.publish(bbox_msg)
            return

        # Draw bounding boxes on image
        cv_image_copy = cv_image.copy()
        
        for box in boxes.xyxy.tolist():
            x1, y1, x2, y2 = box
            # Convert to int for drawing
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            # Draw rectangle on image
            cv2.rectangle(cv_image_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add to RectMultiArray message
            rect = Rect()
            rect.x = float(x1)
            rect.y = float(y1)
            rect.width = float(x2 - x1)
            rect.height = float(y2 - y1)
            bbox_msg.rects.append(rect)

        self.rects_pub.publish(bbox_msg)
        
        # Publish annotated image
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
