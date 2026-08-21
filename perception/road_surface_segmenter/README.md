# Road Surface Segmenter

Road surface semantic segmentation node using YOLOPv2. This node processes camera images and outputs road surface segmentation masks.

## Overview

The `road_surface_segmenter` node uses the YOLOPv2 model to perform semantic segmentation on road surfaces. YOLOPv2 is a multi-task model that can simultaneously perform:
- Object detection
- Road surface segmentation (driving area)
- Lane line detection

This node specifically focuses on **road surface segmentation** (driving area detection).

## Features

- Real-time road surface segmentation using YOLOPv2
- GPU acceleration support (CUDA compatible)
- CPU fallback for systems without GPU
- Visualization of segmentation results
- ROS 2 integration with standard message types

## Output Topics

- `pub_road_surface_mask` (sensor_msgs/Image): Grayscale mask where 1 = road surface, 0 = non-road
- `pub_annotated_image` (sensor_msgs/Image): Original image with green overlay showing detected road surface

## Input Topics

- `sub_image` (sensor_msgs/Image): Input camera image (BGR format)

## Configuration

Edit `config/road_surface_segmenter.yaml` to configure:

```yaml
road_surface_segmenter:
  ros__parameters:
    use_device: 'cpu'  # 'cpu' or '0' for GPU:0
    weight_path: '/path/to/yolopv2/weights/yolopv2.pt'  # Path to YOLOPv2 weights
    normalization:
      mean: 0.0
      standard_deviation: 1.0
```

## Launch

```bash
ros2 launch road_surface_segmenter road_surface_segmenter.launch.py
```

## Dependencies

- torch >= 1.7.0
- torchvision >= 0.8.0
- numpy >= 1.18.5
- opencv-python >= 4.1.1
- ROS 2 (Foxy or later)
- cv_bridge
- sensor_msgs

## Notes

- The node requires a pre-trained YOLOPv2 model file (`.pt` format)
- For GPU acceleration, ensure CUDA-compatible PyTorch is installed
- The model expects 640x640 input resolution internally (letterbox padding handles different input sizes)
