from typing import Optional
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DetectionFusion(Node):
    """Lane mask fusion node: pixel-wise OR of YOLOPv2 (new) and YOLOP (old) masks."""

    OVERLAY_COLOR = np.array([0, 255, 0], dtype=np.uint8)  # BGR green
    OVERLAY_ALPHA = 0.5

    def __init__(self) -> None:
        super().__init__('detection_fusion')
        self.bridge = CvBridge()
        self._image:    Optional[np.ndarray] = None
        self._mask_old: Optional[np.ndarray] = None

        self._mask_pub = self.create_publisher(Image, 'pub_lane_mask',       10)
        self._anno_pub = self.create_publisher(Image, 'pub_annotated_image', 10)

        self.create_subscription(Image, 'sub_image',    self._cache('_image'),    10)
        self.create_subscription(Image, 'sub_lane_old', self._cache('_mask_old'), 10)
        self.create_subscription(Image, 'sub_lane_new', self._cb_lane_new,        10)
        self.get_logger().info('DetectionFusion ready.')

    def _cache(self, attr: str):
        """Return a callback that decodes and caches a frame into self.<attr>."""
        encoding = 'bgr8' if attr == '_image' else 'mono8'
        def cb(msg: Image) -> None:
            try:
                setattr(self, attr, self.bridge.imgmsg_to_cv2(msg, encoding))
            except CvBridgeError as e:
                self.get_logger().warning(f'{attr} CvBridgeError: {e}')
        return cb

    def _cb_lane_new(self, msg: Image) -> None:
        try:
            mask_new = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        except CvBridgeError as e:
            self.get_logger().warning(f'sub_lane_new CvBridgeError: {e}')
            return

        mask_old = self._mask_old if self._mask_old is not None else np.zeros_like(mask_new)
        if mask_old.shape != mask_new.shape:
            mask_old = cv2.resize(mask_old, mask_new.shape[::-1], interpolation=cv2.INTER_NEAREST)

        fused = cv2.bitwise_or((mask_new > 0).astype(np.uint8),
                               (mask_old > 0).astype(np.uint8))

        out = self.bridge.cv2_to_imgmsg(fused, 'mono8')
        out.header = msg.header
        self._mask_pub.publish(out)

        if self._image is not None:
            anno = self.bridge.cv2_to_imgmsg(self._overlay(self._image.copy(), fused), 'bgr8')
            anno.header = msg.header
            self._anno_pub.publish(anno)

    def _overlay(self, image: np.ndarray, mask: np.ndarray) -> np.ndarray:
        if mask.shape[:2] != image.shape[:2]:
            mask = cv2.resize(mask, image.shape[1::-1], interpolation=cv2.INTER_NEAREST)
        px = mask.astype(bool)
        image[px] = (image[px] * (1 - self.OVERLAY_ALPHA) + self.OVERLAY_COLOR * self.OVERLAY_ALPHA).astype(np.uint8)
        return image


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()