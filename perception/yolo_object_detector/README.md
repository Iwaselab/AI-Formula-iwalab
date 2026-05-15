# yolo_object_detector

YOLO11-based object detection node for AI Formula.

This package publishes bounding boxes as `aiformula_interfaces/msg/RectMultiArray` on the configured bbox topic.

## Behavior

- The default weight file is set in `launch/yolo_object_detector.launch.py`.
- The node searches the installed package `weights/` directory first.
- If the installed copy is missing, it also searches the workspace source tree under `src/**/perception/yolo_object_detector/weights/`.
- If `weight_path` is not provided, startup fails with an explicit error.

## Run

```bash
source install/setup.bash
ros2 launch yolo_object_detector yolo_object_detector.launch.py
```

To use a different model:

```bash
ros2 launch yolo_object_detector yolo_object_detector.launch.py weight_path:=/absolute/path/to/best.pt
```

## Notes

- Put custom weights in `perception/yolo_object_detector/weights/` if you want the source tree to be used.
- The node does not auto-download weights.
