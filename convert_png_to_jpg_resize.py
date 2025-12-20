import os
from PIL import Image

# 変換対象ディレクトリ
target_dir = r"/home/fuga/AIFMovie/nf/orig"
output_size = (640, 360)

for filename in os.listdir(target_dir):
    if filename.lower().endswith('.png'):
        png_path = os.path.join(target_dir, filename)
        jpg_filename = os.path.splitext(filename)[0] + '.jpg'
        jpg_path = os.path.join(target_dir, jpg_filename)

        with Image.open(png_path) as img:
            img.load()  # 画像を完全に読み込む
            img = img.convert('RGB')
            img = img.resize(output_size, resample=Image.LANCZOS)
            img.save(jpg_path, 'JPEG', quality=95)

        # 必要なら元のPNGファイルを削除
        os.remove(png_path)
