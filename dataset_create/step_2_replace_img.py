import os
import shutil


def replace_images(dataset_root, original_images_dir):
    # Walk through all directories in the dataset
    for dirpath, dirnames, filenames in os.walk(dataset_root):
        for filename in filenames:
            if filename.endswith(".jpg"):  # Assuming the images are JPGs
                # Construct full path for the current file
                current_file_path = os.path.join(dirpath, filename)

                # Construct path for the corresponding original file
                original_file_path = os.path.join(original_images_dir, filename)

                # Check if the original file exists
                if os.path.exists(original_file_path):
                    # Replace the file
                    shutil.copy2(original_file_path, current_file_path)
                    print(f"Replaced: {current_file_path}")
                else:
                    print(f"Original not found for: {filename}")


# Specify the root directory of your dataset
dataset_root = "/Users/max/Downloads/warfare_soldier"

# Specify the directory containing the original (uncompressed) images
original_images_dir = "/Users/max/Downloads/extracted_images"

# Call the function
replace_images(dataset_root, original_images_dir)
