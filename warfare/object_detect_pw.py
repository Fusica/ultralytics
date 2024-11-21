#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_matrix
from ultralytics import YOLO
import logging

LOGGER = logging.getLogger(__name__)


class ObjectLocalizer:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("object_localizer")

        # 创建CV桥接器
        self.bridge = CvBridge()

        # 加载YOLO模型
        self.yolo = YOLO("yolov8n.pt")

        # 保持标定的相机内参
        self.camera_matrix = np.array([[1057.94375967, 0, 932.90751089], [0, 1057.01503857, 536.51989509], [0, 0, 1]])

        # 使用厂商提供的FOV参数
        self.fov_horizontal = 81  # 水平FOV
        self.fov_vertical = 53  # 垂直FOV
        self.fov_diagonal = 93  # 对角FOV

        # 实际使用的图像分辨率
        self.image_width = 1920  # 1080p
        self.image_height = 1080

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
        self.current_frame = None

        # 订阅器
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_callback)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # 发布器
        self.object_pub = rospy.Publisher("/detected_objects", Float32MultiArray, queue_size=10)

        # 设置处理频率
        self.rate = rospy.Rate(30)  # 30Hz

    def drone_pose_callback(self, msg):
        """处理无人机位姿数据"""
        self.current_drone_pose = msg

    def image_callback(self, msg):
        """处理图像数据"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame()
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def estimate_distance(self, pixel_y):
        """估算目标距离"""
        if self.current_drone_pose is None:
            return None

        # 使用垂直FOV进行计算
        center_y = self.image_height / 2
        degrees_per_pixel = self.fov_vertical / self.image_height
        pixel_angle = (pixel_y - center_y) * degrees_per_pixel

        # 添加调试信息
        rospy.loginfo(f"pixel_y: {pixel_y}, angle: {pixel_angle}")

        # 获取无人机姿态
        orientation = self.current_drone_pose.pose.orientation
        _, pitch, _ = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # 获取无人机高度
        drone_height = -self.current_drone_pose.pose.position.z

        # 转换为弧度
        pixel_angle_rad = np.deg2rad(pixel_angle)
        camera_angle = pitch + pixel_angle_rad

        # 通过三角关系计算距离
        if abs(camera_angle) < np.pi / 2:  # 避免除以零或负值
            distance = drone_height / np.tan(camera_angle)
            return abs(distance)
        return None

    def process_frame(self):
        """处理当前帧"""
        if self.current_frame is None or self.current_drone_pose is None:
            return

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
        results = self.yolo(self.current_frame)[0]

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

            # 在process_frame中添加调试信息
            # 相机坐标系下的位置
            x = (center_x - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0] * distance
            y = (center_y - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1] * distance
            z = distance
            rospy.loginfo(f"Camera coords: x={x}, y={y}, z={z}")

            # 相机到机体坐标系
            p_C = np.array([[x], [y], [z]])
            p_B = self.R_B_C @ p_C + self.t_B_C
            rospy.loginfo(f"Body coords: x={p_B[0][0]}, y={p_B[1][0]}, z={p_B[2][0]}")

            # 机体到世界坐标系
            p_W = R_W_B @ p_B + drone_pos
            rospy.loginfo(f"World coords: x={p_W[0][0]}, y={p_W[1][0]}, z={p_W[2][0]}")

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
        """运行节点"""
        rospy.spin()


def main():
    try:
        localizer = ObjectLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
