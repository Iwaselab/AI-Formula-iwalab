import os
from PIL import Image

def reverse_images_in_dir(input_dir):
    output_dir = "/home/fuga/AIFMovie/images/mask"
    os.makedirs(output_dir, exist_ok=True)
    for fname in os.listdir(input_dir):
        if fname.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            img_path = os.path.join(input_dir, fname)
            img = Image.open(img_path)
            reversed_img = img.transpose(Image.FLIP_LEFT_RIGHT)
            out_fname = 'r-' + fname
            out_path = os.path.join(output_dir, out_fname)
            reversed_img.save(out_path)
            print(f"Saved: {out_path}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python reverse_images.py <input_dir>")
    else:
        input_dir = sys.argv[1]
        reverse_images_in_dir(input_dir)
