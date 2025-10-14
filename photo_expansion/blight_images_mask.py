import os
import shutil

def batch_rename_images(src_dir, dst_dir, prefix="b-"):
    # 出力先ディレクトリがなければ作成
    os.makedirs(dst_dir, exist_ok=True)
    # 画像拡張子リスト
    img_exts = (".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff", ".webp")
    for fname in os.listdir(src_dir):
        if fname.lower().endswith(img_exts):
            src_path = os.path.join(src_dir, fname)
            dst_fname = prefix + fname
            dst_path = os.path.join(dst_dir, dst_fname)
            shutil.copy2(src_path, dst_path)
            print(f"Copied: {src_path} -> {dst_path}")

if __name__ == "__main__":
    src_dir = "/home/fuga/AIFMovie/images/mask"
    dst_dir = "/home/fuga/AIFMovie/images/mask"
    batch_rename_images(src_dir, dst_dir)
