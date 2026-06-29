#!/usr/bin/env python
import os
import random
import re
 
data_dir = '/home/iwalab/AIFMovie/crosswalk/images'
labels_dir = '/home/iwalab/AIFMovie/crosswalk/labels'
train_data_dir = 'train'
valid_data_dir = 'valid'
test_data_dir = 'test'
 
# Get all image files
image_files = [
    f for f in os.listdir(data_dir) if os.path.isfile(os.path.join(data_dir, f))
]
# Get all label files
label_files = [
    f for f in os.listdir(labels_dir) if os.path.isfile(os.path.join(labels_dir, f))
]
 
# Extract base names (without extension) for matching
image_names = {}
for img_file in image_files:
    match = re.match(r"(.*).(jpg|png)", img_file)
    if match:
        base_name = match.group(1)
        image_names[base_name] = img_file
 
label_names = set()
for label_file in label_files:
    match = re.match(r"(.*).txt", label_file)
    if match:
        base_name = match.group(1)
        label_names.add(base_name)
 
# Find files that exist in both directories
matched_files = []
for base_name in image_names:
    if base_name in label_names:
        matched_files.append((base_name, image_names[base_name]))
 
print(f"Found {len(matched_files)} files with both images and labels (including empty labels)")
print(f"Total images: {len(image_files)}")
print(f"Total labels: {len(label_files)}")
 
# Shuffle for random split
random.shuffle(matched_files)
 
# Create directories if they don't exist
os.makedirs(f"{train_data_dir}/images", exist_ok=True)
os.makedirs(f"{train_data_dir}/labels", exist_ok=True)
os.makedirs(f"{valid_data_dir}/images", exist_ok=True)
os.makedirs(f"{valid_data_dir}/labels", exist_ok=True)
os.makedirs(f"{test_data_dir}/images", exist_ok=True)
os.makedirs(f"{test_data_dir}/labels", exist_ok=True)
 
# Calculate split points (60% train, 20% valid, 20% test)
n = len(matched_files)
train_end = n * 6 // 10
valid_end = n * 8 // 10
 
# Copy training files
for base_name, image_file in matched_files[:train_end]:
    os.system(f"cp {data_dir}/{image_file} {train_data_dir}/images/")
    os.system(f"cp {labels_dir}/{base_name}.txt {train_data_dir}/labels/")
print(f"Copied {train_end} files to training set")
 
# Copy validation files
for base_name, image_file in matched_files[train_end:valid_end]:
    os.system(f"cp {data_dir}/{image_file} {valid_data_dir}/images/")
    os.system(f"cp {labels_dir}/{base_name}.txt {valid_data_dir}/labels/")
print(f"Copied {valid_end - train_end} files to validation set")
 
# Copy test files
for base_name, image_file in matched_files[valid_end:]:
    os.system(f"cp {data_dir}/{image_file} {test_data_dir}/images/")
    os.system(f"cp {labels_dir}/{base_name}.txt {test_data_dir}/labels/")
print(f"Copied {n - valid_end} files to test set")
 
# Copy classes.txt
if os.path.exists(f"{labels_dir}/../classes.txt"):
    os.system(f"cp {labels_dir}/../classes.txt {train_data_dir}/classes.txt")
    os.system(f"cp {labels_dir}/../classes.txt {valid_data_dir}/classes.txt")
    os.system(f"cp {labels_dir}/../classes.txt {test_data_dir}/classes.txt")
    print("Copied classes.txt to all directories")