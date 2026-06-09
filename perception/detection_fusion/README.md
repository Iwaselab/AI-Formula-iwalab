# detection_fusion

`object_road_detector` (YOLOPv2) と `object_road_detector_old` (YOLOP) の
レーンマスクを統合するROS2パッケージ。

両モデルのマスクを合成して1つのマスクとして出力。

## 実行方法

```bash
ros2 launch detection_fusion detection_fusion.launch.py
```
