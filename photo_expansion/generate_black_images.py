import os
from PIL import Image

# 入力画像ディレクトリ
INPUT_DIR = "/home/fuga/AIFMovie/images/mask"
# 出力ディレクトリ
OUTPUT_DIR = "/home/fuga/AIFMovie/images/black"

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for fname in os.listdir(INPUT_DIR):
        if fname.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            in_path = os.path.join(INPUT_DIR, fname)
            with Image.open(in_path) as img:
                size = img.size
                mode = img.mode
                black_img = Image.new(mode, size, color=0)
                out_path = os.path.join(OUTPUT_DIR, fname)
                black_img.save(out_path)
                print(f"Saved: {out_path}")

if __name__ == "__main__":
    main()
