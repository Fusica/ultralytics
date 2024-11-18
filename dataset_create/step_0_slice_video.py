import cv2
import os
from pathlib import Path
import argparse


def video_to_frames(video_path, output_folder, frame_interval=1):
    # 创建输出文件夹
    Path(output_folder).mkdir(parents=True, exist_ok=True)

    # 打开视频文件
    video = cv2.VideoCapture(video_path)

    if not video.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    # 获取视频的基本信息
    fps = video.get(cv2.CAP_PROP_FPS)
    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frame_count / fps

    print(f"Video FPS: {fps}")
    print(f"Total frames: {frame_count}")
    print(f"Duration: {duration:.2f} seconds")

    frame_number = 0
    saved_count = 0

    while True:
        ret, frame = video.read()
        if not ret:
            break

        if frame_number % frame_interval == 0:
            output_path = os.path.join(output_folder, f"frame_{frame_number:06d}.jpg")
            cv2.imwrite(output_path, frame)
            saved_count += 1

        frame_number += 1

    video.release()
    print(f"Saved {saved_count} frames as PNG images in {output_folder}")


def main():
    parser = argparse.ArgumentParser(description="Convert video to PNG frames")
    parser.add_argument(
        "video_path",
        nargs="?",
        default="/Volumes/Data_WD/弹群检测/4k采集数据/tank_video_4k/3.mkv",
        help="Path to the input video file",
    )
    parser.add_argument("output_folder", nargs="?", default="slice_3", help="Path to the output folder for PNG frames")
    parser.add_argument("-i", "--interval", type=int, default=1, help="Frame interval (default: 1)")

    args = parser.parse_args()

    video_to_frames(args.video_path, args.output_folder, args.interval)


if __name__ == "__main__":
    main()
