import os
import json

# 画像フォルダのパス
IMAGE_DIR = "/home/fuga/AIFMovie/images/original"
# 出力先フォルダのパス
OUTPUT_DIR = "/home/fuga/AIFMovie/images/json"

# 画像拡張子のリスト
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".gif")

def main():
    # 出力先フォルダが存在しない場合は作成
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for filename in os.listdir(IMAGE_DIR):
        if filename.lower().endswith(IMAGE_EXTS):
            base_name = os.path.splitext(filename)[0]
            json_filename = f"{base_name}.json"
            json_path = os.path.join(OUTPUT_DIR, json_filename)
            # ここで画像ファイル名を含むjson_contentを作成
            json_content = {
                "name": filename,  # 画像ファイル名（拡張子込み）
                "frames": [
                    {
                        "objects": [
                        ]
                    }
                ]
            }
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(json_content, f, ensure_ascii=False, indent=4)

if __name__ == "__main__":
    main()
