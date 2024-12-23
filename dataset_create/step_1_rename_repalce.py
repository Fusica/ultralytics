import os
import re
import shutil

def find_original_file(original_dir, filename):
    """递归查找原始文件"""
    for dirpath, _, filenames in os.walk(original_dir):
        if filename in filenames:
            return os.path.join(dirpath, filename)
    return None

def replace_files(processed_dir, original_dir):
    pattern = re.compile(r"(.+?)_(png|jpg)\..*\.(jpg|txt)")

    for dirpath, dirnames, filenames in os.walk(processed_dir):
        for filename in filenames:
            if filename.endswith((".jpg", ".txt")):
                match = pattern.match(filename)
                if match:
                    original_name = match.group(1)  # 原始文件名
                    original_ext = match.group(2)  # 原始扩展名 (png或jpg)
                    current_ext = match.group(3)  # 当前文件扩展名 (jpg或txt)

                    if current_ext == "jpg":
                        # 对于图片文件，使用原始扩展名
                        original_filename = f"{original_name}.{original_ext}"
                        new_name = original_filename
                    else:
                        # 对于txt文件，使用与图片相同的名字
                        new_name = f"{original_name}.{current_ext}"

                    current_file = os.path.join(dirpath, filename)
                    new_file = os.path.join(dirpath, new_name)

                    if current_ext == "jpg":
                        original_file = find_original_file(original_dir, original_filename)
                        if original_file:
                            shutil.copy2(original_file, new_file)
                            os.remove(current_file)
                            print(f"Replaced: {filename} -> {new_name}")
                        else:
                            print(f"Warning: Original file not found for {original_filename}")
                    else:
                        os.rename(current_file, new_file)
                        print(f"Renamed label: {filename} -> {new_name}")

# 使用示例
processed_directory = "/home/max/ultralytics/dataset_create/warfare"
original_directory = "/home/max/ultralytics/dataset_create/images"

replace_files(processed_directory, original_directory)
