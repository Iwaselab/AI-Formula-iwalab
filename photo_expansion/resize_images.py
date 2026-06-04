import os
from PIL import Image

# 画像ディレクトリのパス
img_dir = '/home/fuga/AIFMovie/nf/orig'
target_size = (640, 360)

for filename in os.listdir(img_dir):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
        img_path = os.path.join(img_dir, filename)
        with Image.open(img_path) as img:
            resized_img = img.resize(target_size, Image.LANCZOS)
            resized_img.save(img_path)
