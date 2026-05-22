from copy import deepcopy
from typing import Tuple

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from utils.utils import (
    select_device,
    split_for_trace_model,
    non_max_suppression,
    scale_coords,
    letterbox,
    driving_area_mask,
    lane_line_mask,
)

from aiformula_interfaces.msg import RectMultiArray
from common_python.get_ros_parameter import get_ros_parameter
from .object_road_detector_util import to_rect, draw_lane_lines, draw_bounding_boxes


class ObjectRoadDetector(Node):

    def __init__(self):
        super().__init__('object_road_detector')
        device, path_to_weights, mean, stdev = self.get_params()
        self.init_detector(device, path_to_weights, mean, stdev)
        self.cv_bridge = CvBridge()
        buffer_size = 10
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.annotated_image_pub = self.create_publisher(Image, 'pub_annotated_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_bbox', buffer_size)

    def get_params(self) -> Tuple[str, str, float, float]:
        device = str(get_ros_parameter(self, 'use_device'))
        path_to_weights = get_ros_parameter(self, 'weight_path')
        mean = get_ros_parameter(self, 'normalization.mean')
        stdev = get_ros_parameter(self, 'normalization.standard_deviation')
        self.confidence_threshold = get_ros_parameter(self, 'confidence_threshold')
        self.iou_threshold = get_ros_parameter(self, 'iou_threshold')
        # New param: allow disabling object detection when using an external YOLO node
        self.enable_object_detection = bool(get_ros_parameter(self, 'enable_object_detection'))
        return device, path_to_weights, mean, stdev

    def init_detector(self, device: str, path_to_weights: str, mean: float, stdev: float) -> None:
        self.load_detector(device, path_to_weights)
        # Store normalization parameters for potential use
        self.normalization_mean = mean
        self.normalization_stdev = stdev

    def load_detector(self, device: str, path_to_weights: str) -> None:
        self.use_device = select_device(device=device)
        self.use_half_precision = (self.use_device.type != 'cpu')  # half precision only supported on CUDA
        # YOLOPv2 uses TorchScript JIT model
        self.detector = torch.jit.load(path_to_weights)
        self.detector = self.detector.to(self.use_device)
        if self.use_half_precision:
            self.detector = self.detector.half()  # to FP16
        self.detector.eval()

    def image_callback(self, msg: Image) -> None:
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
        
        input_image_size = img_tensor.shape[2:]
        
        # Inference
        with torch.no_grad():
            # YOLOPv2 output: [pred, anchor_grid], seg, ll
            [pred, anchor_grid], seg, ll = self.detector(img_tensor)
            
            # Process YOLOv3 trace output
            pred = split_for_trace_model(pred, anchor_grid)
        
        # Extract lane line mask
        ll_seg_mask = lane_line_mask(ll)
        ll_seg_mask_resized = self.decode_lane_line_output(
            ll_seg_mask, undistorted_image.shape, ratio, (dw, dh))
        
        # Publish lane results always
        self.publish_lane_line(ll_seg_mask_resized, msg.header)

        # Publish objects only if enabled (allows external YOLO node to handle detection)
        if self.enable_object_detection:
            bbox_detections = self.decode_object_output(pred)
            self.publish_rects(input_image_size, undistorted_image.shape, deepcopy(bbox_detections), msg.header)
            self.publish_result_image(undistorted_image, input_image_size, ll_seg_mask_resized,
                                      bbox_detections, msg.header)
        else:
            # When object detection is disabled, still publish annotated image with lane drawings
            self.publish_result_image(undistorted_image, input_image_size, ll_seg_mask_resized,
                                      torch.zeros((0, 6)), msg.header)

    def decode_lane_line_output(self, ll_seg_mask: np.ndarray, original_image_shape: Tuple[int, int, int], ratio: Tuple[float, float], padding: Tuple[float, float]) -> np.ndarray:
        """
        Resize lane line segmentation mask to original image size.
        
        Args:
            ll_seg_mask: Lane line segmentation mask from YOLOPv2 (already resized by lane_line_mask)
            original_image_shape: Shape of original image (height, width, channels)
            ratio: Scale ratio (r, r) from letterbox
            padding: Padding values (dw, dh) from letterbox
        
        Returns:
            Resized lane line mask matching original image size
        """
        # ll_seg_mask is already processed by lane_line_mask(), need to reverse letterbox operation
        h_orig, w_orig = original_image_shape[:2]
        
        # Remove padding and resize to original size
        # Note: lane_line_mask already handles some resizing, just resize to original dimensions
        ll_seg_mask_resized = cv2.resize(ll_seg_mask.astype(np.uint8), (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
        return ll_seg_mask_resized

    def decode_object_output(self, pred: torch.Tensor) -> torch.Tensor:
        """
        Apply NMS to predictions from YOLOPv2.
        
        Args:
            pred: Predictions tensor from split_for_trace_model
        
        Returns:
            Detections tensor after NMS
        """
        batched_detections = non_max_suppression(
            pred.unsqueeze(0) if pred.ndim == 2 else pred,
            conf_thres=self.confidence_threshold,
            iou_thres=self.iou_threshold,
            classes=None,
            agnostic=False,
        )
        FIRST_IMAGE_INDEX = 0
        return batched_detections[FIRST_IMAGE_INDEX]

    def publish_lane_line(self, ll_seg_mask: np.ndarray, header: Header) -> None:
        ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(ll_seg_mask, 'mono8')
        ll_seg_mask_msg.header = header
        self.lane_mask_image_pub.publish(ll_seg_mask_msg)

    def publish_rects(self, input_image_shape: torch.Size, undistorted_image_shape: Tuple[int, int, int], bbox_detections: torch.Tensor, header: Header) -> None:
        # Extract the bounding box coordinates: top-left, bottom-right
        bbox_coords = scale_coords(input_image_shape, bbox_detections[:, :4], undistorted_image_shape).round()
        bbox_msg = RectMultiArray()
        for bbox_coord in bbox_coords:
            bbox_msg.rects.append(to_rect(bbox_coord))
        bbox_msg.header = header
        self.rects_pub.publish(bbox_msg)

    def publish_result_image(self, undistorted_image: np.ndarray, input_image_shape: torch.Size, ll_seg_mask: np.ndarray, bbox_detections: torch.Tensor, header: Header) -> None:
        draw_lane_lines(undistorted_image, ll_seg_mask)
        draw_bounding_boxes(undistorted_image, bbox_detections, input_image_shape)
        annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(undistorted_image, 'bgr8')
        annotated_image_msg.header = header
        self.annotated_image_pub.publish(annotated_image_msg)


def main(args=None):
    rclpy.init(args=None)
    object_road_detector = ObjectRoadDetector()
    try:
        rclpy.spin(object_road_detector)
    except KeyboardInterrupt:
        print('Caught KeyboardInterrupt (Ctrl+C), shutting down...')
    finally:
        object_road_detector.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
