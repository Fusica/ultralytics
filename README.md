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


