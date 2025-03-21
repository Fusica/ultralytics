# Summary of project codes such as strike detection based on YOLO implementation

A ROS-based project for object detection.

## Overview

This project implements a series of detection scripts, the main features are as follows:
- Real-time enemy detection using YOLOv8, return target position in body-frame
- Real-time tod detection using YOLOv8, return the width between the two tods (not considering the case where the camera plane is not parallel to the tod plane)
- Real-time detection of four types of objects on the battlefield, and use height and RGB to calculate the object position in body-frame

## System Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- CUDA-capable GPU (recommended)
- Realsense D435i
- YDLidar (SDM18)

### Dependencies

- ROS Noetic
- OpenCV
- PyTorch
- Ultralytics YOLOv8
- NumPy
- PX4 Autopilot
- YDLidar SDK

# 图片转视频工具

这个简单的Python脚本可以将文件夹中的图片序列转换为MP4格式的视频。

## 功能

- 支持多种图片格式（JPG, JPEG, PNG）
- 可自定义视频帧率
- 自动调整所有图片尺寸以匹配第一张图片
- 显示进度条

## 安装依赖

```bash
pip install -r requirements.txt
```

## 使用方法

基本用法：

```bash
python create_video.py
```

这将使用默认设置：
- 输入文件夹: `./images`
- 输出文件: `output_video.mp4`
- 帧率: 30 fps
- 图片格式: `*.jpg,*.jpeg,*.png`

### 自定义参数

所有参数都是可选的：

```bash
python create_video.py --input_folder 你的图片文件夹 --output_file 输出视频名称 --fps 24 --pattern "*.png"
```

可用参数：
- `--input_folder`: 设置输入图片文件夹（默认：`./images`）
- `--output_file`: 设置输出视频文件名（默认：`output_video.mp4`）
- `--fps`: 设置视频帧率（默认：30）
- `--pattern`: 设置图片文件匹配模式（默认：`*.jpg,*.jpeg,*.png`）

例如，只修改输出文件名：
```bash
python create_video.py --output_file my_video.mp4
```

或只修改帧率：
```bash
python create_video.py --fps 60
```

## 注意事项

- 图片会按文件名排序
- 所有图片将被调整为与第一张图片相同的尺寸
- 如果输出文件名没有`.mp4`后缀，会自动添加


