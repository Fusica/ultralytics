#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf.transformations import euler_from_quaternion, quaternion_matrix
from ultralytics import YOLO
from threading import Thread
from pathlib import Path
import time
import torch
import math
from urllib.parse import urlparse
import os
import logging

LOGGER = logging.getLogger(__name__)


class LoadStreams:
    """
    Stream Loader for various types of video streams, Supports RTSP, RTMP, HTTP, and TCP streams.
    """

    def __init__(self, sources="file.streams", vid_stride=1, buffer=False):
        """Initialize instance variables and check for consistent input stream shapes."""
        torch.backends.cudnn.benchmark = True  # faster for fixed-size inference
        self.buffer = buffer  # buffer input streams
        self.running = True  # running flag for Thread
        self.mode = "stream"
        self.vid_stride = vid_stride  # video frame-rate stride

        sources = Path(sources).read_text().rsplit() if os.path.isfile(sources) else [sources]
        n = len(sources)
        self.bs = n
        self.fps = [0] * n  # frames per second
        self.frames = [0] * n
        self.threads = [None] * n
        self.caps = [None] * n  # video capture objects
        self.imgs = [[] for _ in range(n)]  # images
        self.shape = [[] for _ in range(n)]  # image shapes
        self.sources = [s for s in sources]  # clean source names for later
        for i, s in enumerate(sources):  # index, source
            st = f"{i + 1}/{n}: {s}... "
            s = eval(s) if s.isnumeric() else s  # i.e. s = '0' local webcam
            self.caps[i] = cv2.VideoCapture(s)  # store video capture object
            if not self.caps[i].isOpened():
                raise ConnectionError(f"{st}Failed to open {s}")
            w = int(self.caps[i].get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.caps[i].get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.caps[i].get(cv2.CAP_PROP_FPS)  # warning: may return 0 or nan
            self.frames[i] = max(int(self.caps[i].get(cv2.CAP_PROP_FRAME_COUNT)), 0) or float("inf")
            self.fps[i] = max((fps if math.isfinite(fps) else 0) % 100, 0) or 30  # 30 FPS fallback

            success, im = self.caps[i].read()  # guarantee first frame
            if not success or im is None:
                raise ConnectionError(f"{st}Failed to read images from {s}")
            self.imgs[i].append(im)
            self.shape[i] = im.shape
            self.threads[i] = Thread(target=self.update, args=([i, self.caps[i], s]), daemon=True)
            LOGGER.info(f"{st}Success ✅ ({self.frames[i]} frames of shape {w}x{h} at {self.fps[i]:.2f} FPS)")
            self.threads[i].start()
        LOGGER.info("")  # newline

    def update(self, i, cap, stream):
        """Read stream `i` frames in daemon thread."""
        n, f = 0, self.frames[i]  # frame number, frame array
        while self.running and cap.isOpened() and n < (f - 1):
            if len(self.imgs[i]) < 30:  # keep a <=30-image buffer
                n += 1
                cap.grab()  # .read() = .grab() followed by .retrieve()
                if n % self.vid_stride == 0:
                    success, im = cap.retrieve()
                    if not success:
                        im = np.zeros(self.shape[i], dtype=np.uint8)
                        LOGGER.warning("WARNING ⚠️ Video stream unresponsive, please check your IP camera connection.")
                        cap.open(stream)  # re-open stream if signal was lost
                    if self.buffer:
                        self.imgs[i].append(im)
                    else:
                        self.imgs[i] = [im]
            else:
                time.sleep(0.01)  # wait until the buffer is empty

    def close(self):
        """Close stream loader and release resources."""
        self.running = False  # stop flag for Thread
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=5)  # Add timeout
        for cap in self.caps:  # Iterate through the stored VideoCapture objects
            try:
                cap.release()  # release video capture
            except Exception as e:
                LOGGER.warning(f"WARNING ⚠️ Could not release VideoCapture object: {e}")
        cv2.destroyAllWindows()

    def __iter__(self):
        """Iterates through YOLO image feed and re-opens unresponsive streams."""
        self.count = -1
        return self

    def __next__(self):
        """Returns source paths, transformed and original images for processing."""
        self.count += 1

        images = []
        for i, x in enumerate(self.imgs):
            # Wait until a frame is available in each buffer
            while not x:
                if not self.threads[i].is_alive() or cv2.waitKey(1) == ord("q"):  # q to quit
                    self.close()
                    raise StopIteration
                time.sleep(1 / min(self.fps))
                x = self.imgs[i]
                if not x:
                    LOGGER.warning(f"WARNING ⚠️ Waiting for stream {i}")

            # Get and remove the first frame from imgs buffer
            if self.buffer:
                images.append(x.pop(0))

            # Get the last frame, and clear the rest from the imgs buffer
            else:
                images.append(x.pop(-1) if x else np.zeros(self.shape[i], dtype=np.uint8))
                x.clear()

        return self.sources, images, [""] * self.bs

    def __len__(self):
        """Return the length of the sources object."""
        return self.bs


class ObjectLocalizer:
    def __init__(self, sources):
        # 初始化视频流加载器
        self.stream_loader = LoadStreams(sources)

        # 加载YOLO模型
        self.yolo = YOLO("/home/max/ultralytics/runs/detect/tank/weights/best.pt")

        # 相机内参 (需要根据实际相机参数调整)
        self.camera_matrix = np.array(
            [
                [1057.94375967, 0, 932.90751089],
                [0, 1057.01503857, 536.51989509],
                [0, 0, 1],
            ]
        )

        # 相机到机体的变换
        self.R_B_C = np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ]
        )
        self.t_B_C = np.array([[0.36], [0.0], [0.0]])

        self.current_drone_pose = None

        # 订阅无人机位姿
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_callback)

        # 修改发布器为Float32MultiArray类型
        self.object_pub = rospy.Publisher("/detected_objects", Float32MultiArray, queue_size=10)

        # 设置处理频率
        self.rate = rospy.Rate(30)  # 30Hz

    def drone_pose_callback(self, msg):
        self.current_drone_pose = msg

    def estimate_distance(self, pixel_y, fov_vertical=60):
        """估算目标距离"""
        if self.current_drone_pose is None:
            return None

        # 获取无人机姿态
        orientation = self.current_drone_pose.pose.orientation
        _, pitch, _ = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # 获取无人机高度
        drone_height = -self.current_drone_pose.pose.position.z  # 假设向上为负

        # 计算目标在图像中的垂直角度
        image_height = 480  # 根据实际图像大小调整
        center_y = image_height / 2
        degrees_per_pixel = fov_vertical / image_height
        pixel_angle = (pixel_y - center_y) * degrees_per_pixel

        # 转换为弧度
        pixel_angle_rad = np.deg2rad(pixel_angle)
        camera_angle = pitch + pixel_angle_rad

        # 通过三角关系计算距离
        if abs(camera_angle) < np.pi / 2:  # 避免除以零或负值
            distance = drone_height / np.tan(camera_angle)
            return abs(distance)
        return None

    def process_stream(self):
        for _, frames, _ in self.stream_loader:
            if self.current_drone_pose is None:
                continue

            for frame in frames:
                # 获取无人机的位姿
                drone_pos = np.array(
                    [
                        [self.current_drone_pose.pose.position.x],
                        [self.current_drone_pose.pose.position.y],
                        [self.current_drone_pose.pose.position.z],
                    ]
                )

                q = self.current_drone_pose.pose.orientation
                R_W_B = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

                # YOLO检测
                results = self.yolo(frame)[0]

                # 创建存储所有检测结果的列表
                detections = []

                # 处理每个检测结果
                for det in results.boxes:
                    if det.conf < 0.5:
                        continue

                    # 获取类别ID
                    class_id = int(det.cls[0])

                    # 获取边界框
                    bbox = det.xyxy[0].cpu().numpy()
                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2

                    # 估算距离
                    distance = self.estimate_distance(center_y)
                    if distance is None:
                        continue

                    # 计算相机坐标系下的位置
                    x = (center_x - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0] * distance
                    y = (center_y - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1] * distance
                    z = distance

                    # 相机坐标系到机体坐标系
                    p_C = np.array([[x], [y], [z]])
                    p_B = self.R_B_C @ p_C + self.t_B_C

                    # 机体坐标系到世界坐标系
                    p_W = R_W_B @ p_B + drone_pos

                    # 将检测结果添加到列表中 [class_id, x, y, z]
                    detections.append([float(class_id), float(p_W[0][0]), float(p_W[1][0]), float(p_W[2][0])])

                # 创建并发布Float32MultiArray消息
                if detections:
                    msg = Float32MultiArray()
                    msg.layout.dim = [
                        MultiArrayDimension(label="rows", size=len(detections), stride=4),
                        MultiArrayDimension(label="cols", size=4, stride=1),
                    ]
                    msg.data = [item for sublist in detections for item in sublist]
                    self.object_pub.publish(msg)

    def run(self):
        try:
            self.process_stream()
        except rospy.ROSInterruptException:
            pass


def main():
    rospy.init_node("object_localizer")
    sources = "rtsp://192.168.1.25:8554/main.264"  # 这里可以更改为不同的流源
    localizer = ObjectLocalizer(sources)
    localizer.run()


if __name__ == "__main__":
    main()
