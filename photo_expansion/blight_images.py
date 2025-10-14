import os
from PIL import Image

def redden_image(img):
    # 赤みを加える（R値を強調）
    r, g, b = img.split()
    r = r.point(lambda i: min(255, int(i * 1.3)))
    return Image.merge('RGB', (r, g, b))

def process_images(input_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    for fname in os.listdir(input_dir):
        if fname.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            path = os.path.join(input_dir, fname)
            img = Image.open(path).convert('RGB')
            img_red = redden_image(img)
            out_fname = 'b-' + fname
            out_path = os.path.join(output_dir, out_fname)
            img_red.save(out_path)
            print(f"Saved: {out_path}")

if __name__ == "__main__":
    input_dir = "/home/fuga/AIFMovie/images/original"
    output_dir = "/home/fuga/AIFMovie/images/original"
    process_images(input_dir, output_dir)
