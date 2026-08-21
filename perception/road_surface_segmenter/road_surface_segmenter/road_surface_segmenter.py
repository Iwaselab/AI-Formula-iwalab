from typing import Tuple

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# Add yolopv2 utils to path
yolopv2_path = Path(__file__).parent.parent.parent / 'yolopv2'
if str(yolopv2_path) not in sys.path:
    sys.path.insert(0, str(yolopv2_path))

from utils.utils import (
    select_device,
    split_for_trace_model,
    letterbox,
    driving_area_mask,
)

from common_python.get_ros_parameter import get_ros_parameter


class RoadSurfaceSegmenter(Node):
    """
    Road surface semantic segmentation node using YOLOPv2.
    Processes camera images and outputs road surface segmentation masks.
    """

    def __init__(self):
        super().__init__('road_surface_segmenter')
        device, path_to_weights, mean, stdev = self.get_params()
        self.init_detector(device, path_to_weights, mean, stdev)
        self.cv_bridge = CvBridge()
        buffer_size = 10
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.road_surface_mask_pub = self.create_publisher(Image, 'pub_road_surface_mask', buffer_size)
        self.annotated_image_pub = self.create_publisher(Image, 'pub_annotated_image', buffer_size)

    def get_params(self) -> Tuple[str, str, float, float]:
        """Get parameters from ROS config."""
        device = str(get_ros_parameter(self, 'use_device'))
        path_to_weights = get_ros_parameter(self, 'weight_path')
        mean = get_ros_parameter(self, 'normalization.mean')
        stdev = get_ros_parameter(self, 'normalization.standard_deviation')
        return device, path_to_weights, mean, stdev

    def init_detector(self, device: str, path_to_weights: str, mean: float, stdev: float) -> None:
        """Initialize the YOLOPv2 detector."""
        self.load_detector(device, path_to_weights)
        self.normalization_mean = mean
        self.normalization_stdev = stdev

    def load_detector(self, device: str, path_to_weights: str) -> None:
        """Load YOLOPv2 TorchScript model."""
        self.use_device = select_device(device=device)
        self.use_half_precision = (self.use_device.type != 'cpu')
        # YOLOPv2 uses TorchScript JIT model
        self.detector = torch.jit.load(path_to_weights)
        self.detector = self.detector.to(self.use_device)
        if self.use_half_precision:
            self.detector = self.detector.half()  # to FP16
        self.detector.eval()
        self.get_logger().info(f'Loaded YOLOPv2 model from {path_to_weights}')

    def image_callback(self, msg: Image) -> None:
        """Process incoming image and extract road surface segmentation."""
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warning(f"CvBridgeError occurred: {str(e)}")
            return
        
        # Padded resize using letterbox
        img_size = 640
        stride = 32
        padded_image, ratio, (dw, dh) = letterbox(undistorted_image, img_size, stride=stride, auto=True)
        
        # Convert image to tensor (BGR to RGB, and normalize)
        img_tensor = padded_image[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
        img_tensor = np.ascontiguousarray(img_tensor)
        img_tensor = torch.from_numpy(img_tensor).to(self.use_device)
        img_tensor = img_tensor.half() if self.use_half_precision else img_tensor.float()
        img_tensor = img_tensor / 255.0  # 0-255 to 0.0-1.0
        
        # Add batch dimension
        if img_tensor.ndimension() == 3:
            img_tensor = img_tensor.unsqueeze(0)
        
        # Inference
        with torch.no_grad():
            # YOLOPv2 output: [pred, anchor_grid], seg, ll
            # seg: driving area (road surface) segmentation
            [pred, anchor_grid], seg, ll = self.detector(img_tensor)
        
        # Extract road surface segmentation mask
        da_seg_mask = driving_area_mask(seg)
        da_seg_mask_resized = self.decode_road_surface_output(
            da_seg_mask, undistorted_image.shape, ratio, (dw, dh))
        
        # Publish results
        self.publish_road_surface_mask(da_seg_mask_resized, msg.header)
        self.publish_annotated_image(undistorted_image, da_seg_mask_resized, msg.header)

    def decode_road_surface_output(self, da_seg_mask: np.ndarray, original_image_shape: Tuple[int, int, int], ratio: Tuple[float, float], padding: Tuple[float, float]) -> np.ndarray:
        """
        Resize road surface segmentation mask to original image size.
        
        Args:
            da_seg_mask: Driving area segmentation mask from YOLOPv2
            original_image_shape: Shape of original image (height, width, channels)
            ratio: Scale ratio (r, r) from letterbox
            padding: Padding values (dw, dh) from letterbox
        
        Returns:
            Resized road surface mask matching original image size
        """
        h_orig, w_orig = original_image_shape[:2]
        
        # Resize to original size
        da_seg_mask_resized = cv2.resize(da_seg_mask.astype(np.uint8), (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
        return da_seg_mask_resized

    def publish_road_surface_mask(self, da_seg_mask: np.ndarray, header: Header) -> None:
        """Publish the road surface segmentation mask."""
        da_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(da_seg_mask, 'mono8')
        da_seg_mask_msg.header = header
        self.road_surface_mask_pub.publish(da_seg_mask_msg)

    def publish_annotated_image(self, undistorted_image: np.ndarray, da_seg_mask: np.ndarray, header: Header) -> None:
        """Publish annotated image with road surface overlay."""
        # Create a colored visualization of the road surface mask
        annotated_image = undistorted_image.copy()
        
        # Apply colormap to the mask for visualization
        # Values: 0 = not road, 1 = road
        road_visualization = np.zeros_like(annotated_image)
        road_visualization[da_seg_mask == 1] = [0, 255, 0]  # Green for road
        
        # Blend with original image
        alpha = 0.5
        annotated_image = cv2.addWeighted(annotated_image, 1 - alpha, road_visualization, alpha, 0)
        
        # Publish
        annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
        annotated_image_msg.header = header
        self.annotated_image_pub.publish(annotated_image_msg)


def main(args=None):
    rclpy.init(args=None)
    road_surface_segmenter = RoadSurfaceSegmenter()
    try:
        rclpy.spin(road_surface_segmenter)
    except KeyboardInterrupt:
        print('Caught KeyboardInterrupt (Ctrl+C), shutting down...')
    finally:
        road_surface_segmenter.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
