import os
import re


def rename_files(root_dir):
    # Compile the regex pattern to match the file names
    pattern = re.compile(r"(.+?)_jpg\..*\.(jpg|txt)")

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
root_directory = "/Users/max/Downloads/warfare_soldier"

# Call the function
rename_files(root_directory)
