import os
import argparse
import cv2
import numpy as np
from glob import glob
from tqdm import tqdm


def create_video_from_images(input_folder, output_file, fps=30, img_pattern="*.jpg,*.jpeg,*.png", quality=95):
    """
    Create a video from all images in the specified folder.

    Args:
        input_folder (str): Path to the folder containing images
        output_file (str): Path to the output video file
        fps (int): Frames per second for the output video
        img_pattern (str): Comma-separated patterns for image files to include
        quality (int): Video quality (0-100, higher means better quality but larger file, 95 is default)
    """
    # Get all image files in the folder
    patterns = img_pattern.split(",")
    image_files = []
    for pattern in patterns:
        image_files.extend(glob(os.path.join(input_folder, pattern.strip())))

    # Sort the image files to ensure proper sequence
    image_files.sort()

    if not image_files:
        print(f"No images found in {input_folder} matching pattern {img_pattern}")
        return

    # Read the first image to get dimensions
    first_img = cv2.imread(image_files[0])
    if first_img is None:
        print(f"Failed to read image: {image_files[0]}")
        return

    height, width, _ = first_img.shape

    # Create VideoWriter object with quality settings
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    # Set video quality (only works with certain codecs)
    video_writer.set(cv2.VIDEOWRITER_PROP_QUALITY, quality)

    # Add each image to the video
    print(f"Creating video from {len(image_files)} images...")
    for img_path in tqdm(image_files):
        img = cv2.imread(img_path)
        if img is not None:
            # Ensure image has the same dimensions as the first image
            if img.shape[0] != height or img.shape[1] != width:
                img = cv2.resize(img, (width, height))
            # 可选：降低图片质量以减小文件大小
            if quality < 90:
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
                _, img = cv2.imencode(".jpg", img, encode_param)
                img = cv2.imdecode(img, 1)
            video_writer.write(img)
        else:
            print(f"Warning: Could not read image {img_path}")

    # Release the video writer
    video_writer.release()
    print(f"Video saved to {output_file}")


def main():
    parser = argparse.ArgumentParser(description="Create a video from images in a folder")
    parser.add_argument(
        "--input_folder",
        default="/Users/max/Downloads/predict20",
        help="Path to the folder containing images (default: ./images)",
    )
    parser.add_argument(
        "--output_file", default="output_video.mp4", help="Path to the output video file (default: output_video.mp4)"
    )
    parser.add_argument("--fps", type=int, default=30, help="Frames per second (default: 30)")
    parser.add_argument(
        "--pattern",
        default="*.jpg,*.jpeg,*.png",
        help='Comma-separated patterns for image files (default: "*.jpg,*.jpeg,*.png")',
    )
    parser.add_argument(
        "--quality",
        type=int,
        default=50,
        help="Video quality (0-100, higher means better quality but larger file, 95 is default)",
    )

    args = parser.parse_args()

    # Ensure output file has .mp4 extension
    if not args.output_file.lower().endswith(".mp4"):
        args.output_file += ".mp4"

    create_video_from_images(args.input_folder, args.output_file, args.fps, args.pattern, args.quality)


if __name__ == "__main__":
    main()
