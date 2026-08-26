#!/bin/bash
# ROS2 bagファイルから画像を抽出するスクリプト

set -e

# 使用方法を表示する関数
show_usage() {
    echo "使用方法: $0 <bag_directory> <image_topic> [output_directory]"
    echo ""
    echo "例:"
    echo "  $0 /path/to/bag /camera/image_raw ./extracted_images"
    echo ""
    echo "利用可能な画像トピックを確認:"
    echo "  ros2 bag info /path/to/bag"
    exit 1
}

# 引数の確認
if [ $# -lt 2 ]; then
    show_usage
fi

BAG_DIR="$1"
IMAGE_TOPIC="$2"
OUTPUT_DIR="${3:-/home/fuga/AIFMovie/images/extracted}"
PLAY_RATE="${PLAY_RATE:-1.0}"
SAVER_PID=""
CLEANED_UP=false

cleanup() {
    if [[ "$CLEANED_UP" == true ]]; then
        return
    fi
    CLEANED_UP=true
    if [[ -n "$SAVER_PID" ]] && kill -0 "$SAVER_PID" 2>/dev/null; then
        kill -INT "$SAVER_PID" 2>/dev/null || true
        for _ in {1..30}; do
            kill -0 "$SAVER_PID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$SAVER_PID" 2>/dev/null; then
            echo "画像保存ノードが終了しないため強制停止します..." >&2
            kill -TERM "$SAVER_PID" 2>/dev/null || true
            sleep 1
            kill -KILL "$SAVER_PID" 2>/dev/null || true
        fi
        wait "$SAVER_PID" 2>/dev/null || true
    fi
    SAVER_PID=""
    rm -f /tmp/image_saver_node.py
}

trap cleanup EXIT INT TERM

echo "ROS2 bag画像抽出スクリプト"
echo "=========================="
echo "Bagディレクトリ: $BAG_DIR"
echo "画像トピック: $IMAGE_TOPIC"
echo "出力ディレクトリ: $OUTPUT_DIR"
echo ""

# 出力ディレクトリの作成
mkdir -p "$OUTPUT_DIR"

# bagファイルの情報を表示
echo "Bagファイル情報:"
ros2 bag info "$BAG_DIR"
echo ""

# 画像保存ノードを作成（Python実装）
cat > /tmp/image_saver_node.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import queue
import sys
import threading
from pathlib import Path
from rclpy.qos import qos_profile_sensor_data

class ImageSaver(Node):
    def __init__(self, topic_name, output_dir):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.output_dir = Path(output_dir)
        self.saved_count = 0
        self.received_count = 0
        self.dropped_count = 0
        self.message_queue = queue.Queue(maxsize=5000)
        self.stopping = False
        self.worker = threading.Thread(target=self.save_images, daemon=True)
        self.worker.start()
        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(f'画像保存開始: {topic_name} -> {output_dir}')
    
    def image_callback(self, msg):
        try:
            self.message_queue.put_nowait(msg)
            self.received_count += 1
        except Exception as e:
            if isinstance(e, queue.Full):
                self.dropped_count += 1
            else:
                self.get_logger().error(f'受信エラー: {e}')

    def save_images(self):
        while not self.stopping or not self.message_queue.empty():
            try:
                msg = self.message_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                timestamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                filename = f"frame_{self.saved_count:06d}_{timestamp}.jpg"
                filepath = self.output_dir / filename
                if not cv2.imwrite(str(filepath), cv_image):
                    raise RuntimeError('JPEG書き込みに失敗しました')
                self.saved_count += 1
                if self.saved_count == 1 or self.saved_count % 100 == 0:
                    self.get_logger().info(f'保存: {filename} (計{self.saved_count}枚)')
            except Exception as e:
                self.get_logger().error(f'保存エラー: {e}')
            finally:
                self.message_queue.task_done()

    def finish(self):
        self.stopping = True
        self.message_queue.join()
        self.worker.join()

def main():
    rclpy.init()
    topic = sys.argv[1]
    output_dir = sys.argv[2]
    
    saver = ImageSaver(topic, output_dir)
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    finally:
        saver.finish()
        print(f"\n受信枚数: {saver.received_count}")
        print(f"総保存枚数: {saver.saved_count}")
        print(f"キュー満杯による欠落: {saver.dropped_count}")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 画像保存ノードをバックグラウンドで開始
echo "画像保存ノードを開始中..."
python3 /tmp/image_saver_node.py "$IMAGE_TOPIC" "$OUTPUT_DIR" &
SAVER_PID=$!

# 少し待機
sleep 2

# bagファイルを再生（ROS Foxy対応オプションのみ使用）
echo "Bagファイルを再生中...（${PLAY_RATE}倍速）"
ros2 bag play "$BAG_DIR" --topics "$IMAGE_TOPIC" -r "$PLAY_RATE" -s mcap

# 画像保存ノードを停止
echo "画像保存ノードを停止中..."
cleanup

# 結果を表示
echo ""
echo "抽出完了！"
echo "出力先: $OUTPUT_DIR"
echo "保存された画像数:"
ls -1 "$OUTPUT_DIR"/*.jpg 2>/dev/null | wc -l || echo "0"

echo ""
echo "画像ファイル一覧:"
ls -la "$OUTPUT_DIR"/ 2>/dev/null || echo "画像が見つかりません"