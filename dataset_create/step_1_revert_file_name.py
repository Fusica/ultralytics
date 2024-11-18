import os
import re


def rename_files(root_dir):
    # Compile the regex pattern to match the file names
    pattern = re.compile(r"(frame_\d{6}).*\.(jpg|txt)")

    # Walk through all directories
    for dirpath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith((".jpg", ".txt")):
                match = pattern.match(filename)
                if match:
                    # Get the original frame number part and file extension
                    new_name = match.group(1) + "." + match.group(2)
                    old_file = os.path.join(dirpath, filename)
                    new_file = os.path.join(dirpath, new_name)

                    # Rename the file
                    os.rename(old_file, new_file)
                    print(f"Renamed: {filename} -> {new_name}")


# Specify the root directory where your folders are located
root_directory = "/Volumes/Data_WD/弹群检测/4k采集数据/warfare_4k.v2i.yolov8"

# Call the function
rename_files(root_directory)
