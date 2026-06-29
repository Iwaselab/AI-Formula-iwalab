#!/usr/bin/env python3
# fix_class_ids.py
import glob

# 現在ID → 正しいID
ID_MAP = {
    0: 1,  # blue_cone
    1: 0,  # crosswalk
    2: 6,  # go_sign
    3: 3,  # green_cone
    4: 7,  # obstacle
    5: 2,  # red_cone
    6: 5,  # stop_sign
    7: 4,  # yellow_cone
}

label_dirs = [
    '/home/iwalab/AIFMovie/crosswalk/train/labels',
    '/home/iwalab/AIFMovie/crosswalk/valid/labels',
    '/home/iwalab/AIFMovie/crosswalk/test/labels',
]

for label_dir in label_dirs:
    for txt_file in glob.glob(f"{label_dir}/*.txt"):
        lines = open(txt_file).readlines()
        fixed = []
        for line in lines:
            parts = line.strip().split()
            if parts:
                parts[0] = str(ID_MAP[int(parts[0])])
                fixed.append(' '.join(parts))
        open(txt_file, 'w').write('\n'.join(fixed) + '\n')

print("完了")