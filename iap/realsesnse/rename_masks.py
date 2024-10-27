import os


def rename_files(directory):
    # Get all PNG files in the directory
    files = [f for f in os.listdir(directory) if f.endswith('.png')]

    # Sort files to ensure correct order
    files.sort()

    # Rename files
    for i, filename in enumerate(files, start=1):
        old_path = os.path.join(directory, filename)
        new_filename = f"{i}.png"  # New format: 1.png, 2.png, etc.
        new_path = os.path.join(directory, new_filename)

        os.rename(old_path, new_path)
        print(f"Renamed {filename} to {new_filename}")


# Replace this with the path to your directory containing the images
directory = "/Volumes/DATA/Realsense/Video/1727502560/masks"

rename_files(directory)
print("Renaming complete!")