# Warfare Simulation System

A ROS-based warfare simulation system that integrates drone control, object detection, and position tracking.

## Overview

This project implements a warfare simulation system with the following key features:
- Real-time soldier detection using YOLOv8
- 3D position estimation from 2D camera feed
- Drone control and navigation
- ROS-based communication architecture

## System Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- CUDA-capable GPU (recommended)

### Dependencies

- ROS Noetic
- OpenCV
- PyTorch
- Ultralytics YOLOv8
- NumPy
- PX4 Autopilot

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/warfare-simulation.git
cd warfare-simulation
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Build the ROS workspace:
```bash
catkin_make
source devel/setup.bash
```

## Usage

1. Launch the drone simulation:
```bash
roslaunch warfare simulation.launch
```

2. Start the object detection node:
```bash
rosrun warfare object_detect_pb.py
```

3. Launch the control node:
```bash
rosrun warfare control_node.py
```

## System Architecture

### Nodes
- `object_detect_pb.py`: Handles object detection and 3D position estimation
- `control_node.py`: Manages drone control and navigation
- `px4_bridge.py`: Interfaces with PX4 autopilot

### Topics
- `/camera/image_raw`: Raw camera feed
- `/warfare/detections`: Published detection results
- `/mavros/...`: Various MAVLink communication topics

## Coordinate Systems

The system uses the following coordinate frames:
- Camera frame: X-right, Y-down, Z-forward
- Body frame: X-forward, Y-left, Z-up
- World frame: ENU (East-North-Up)

## Configuration

Key parameters can be adjusted in `config/params.yaml`:
```yaml
detection:
  confidence_threshold: 0.5
  model_path: "path/to/model.pt"

camera:
  fx: 1000.0
  fy: 1000.0
  cx: 640.0
  cy: 360.0
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- YOLOv8 by Ultralytics
- PX4 Autopilot Team
- ROS Community

## Contact

Your Name - your.email@example.com
Project Link: [https://github.com/yourusername/warfare-simulation](https://github.com/yourusername/warfare-simulation)
