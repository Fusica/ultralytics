#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import logging
from geometry_msgs.msg import Point, PoseStamped
import math

LOGGER = logging.getLogger(__name__)


class ObjectLocalizer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("object_localizer")

        # 初始化 CV Bridge
        self.bridge = CvBridge()

        # 加载YOLO模型并设置推理大小
        self.yolo = YOLO("/home/jetson/ultralytics/runs/detect/warfare_soldier/weights/best.pt")

        # 相机内参
        self.camera_matrix = np.array(
            [
                [1157.46636105, 0, 972.99380136],
                [0, 1125.00278621, 558.99327223],
                [0, 0, 1],
            ]
        )

        self.dist_coeffs = np.array([0.09693897, -0.02769597, 0.00659544, -0.03210938])
        self.k1 = self.dist_coeffs[0]
        self.k2 = self.dist_coeffs[1]
        self.p1 = self.dist_coeffs[2]
        self.p2 = self.dist_coeffs[3]

        # 提取相机内参
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        # 相机安装角度（度）
        self.camera_pitch = 0.0  # 俯仰角
        self.camera_yaw = 0.0  # 偏航角
        self.camera_roll = 0.0  # 横滚角

        # 计算相机到机体的变换矩阵
        self.update_camera_rotation()

        # TODO 测试使用，真实环境需要去掉直接调用local_position/pose
        self.height = 0.8

        # 发布器和订阅器
        self.object_pub = rospy.Publisher("/detected_objects", Float32MultiArray, queue_size=10)
        self.coordinates_pub = rospy.Publisher("/camera_coordinates", Point, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        self.height_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.height_callback)

    def undistort_pixel(self, pixel_x, pixel_y):
        # 畸变系数
        dist_coeffs = np.array([self.k1, self.k2, self.p1, self.p2])

        # 像素坐标
        pts = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)

        # 去畸变
        undistorted_pts = cv2.undistortPoints(pts, self.camera_matrix, dist_coeffs, P=self.camera_matrix)

        # 提取去畸变后的像素坐标
        undistorted_x = undistorted_pts[0][0][0]
        undistorted_y = undistorted_pts[0][0][1]

        return undistorted_x, undistorted_y

    def calculate_3d_position(self, pixel_x, pixel_y):
        """
        根据物体的像素坐标、相机内参和无人机高度计算物体在相机坐标系下的三维坐标。
        """
        # 获取无人机高度（相机高度）
        h = self.height

        # 去畸变像素坐标
        undistorted_x, undistorted_y = self.undistort_pixel(pixel_x, pixel_y)

        # 计算归一化的相机坐标系方向向量
        x_c = (undistorted_x - self.cx) / self.fx
        y_c = (undistorted_y - self.cy) / self.fy
        z_c = 1  # 归一化后的深度值

        # 计算参数 t，使得射线与地平面（Y=0）相交
        t = h / y_c

        # 计算物体在相机坐标系下的三维坐标
        X = x_c * t
        Y = 0  # 地平面高度
        Z = z_c * t

        position = np.array([X, Y, Z])

        # 发布 Point 消息（可选）
        # point_msg = Point()
        # point_msg.x = float(position[0])
        # point_msg.y = float(position[1])
        # point_msg.z = float(position[2])
        # self.coordinates_pub.publish(point_msg)

        return position

    def height_callback(self, msg):
        """
        处理无人机高度信息
        """
        try:
            # 如果使用 NED 坐标系，z 轴向下为正，需要取反
            self.height = msg.pose.position.z

            rospy.loginfo(f"Current UAV Height: {self.height} meters (NED coordinate system)")
        except Exception as e:
            rospy.logwarn(f"Error processing height: {e}")

    def image_callback(self, msg):
        """处理接收到的图像消息"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLO检测
            results = self.yolo(frame, imgsz=1280)[0]
            detections = []

            # 处理每个检测结果
            for det in results.boxes:
                if det.conf < 0.6:
                    continue

                # 获取类别ID
                class_id = int(det.cls[0])

                # 获取边界框中心点
                bbox = det.xyxy[0].cpu().numpy()
                center_x = (bbox[0] + bbox[2]) / 2
                bottom_y = bbox[3]

                # 计算3D位置（机体坐标系）
                position = self.calculate_3d_position(center_x, bottom_y)
                p_C = position.reshape(3, 1)
                p_B = self.R_B_C @ p_C + self.t_B_C

                # 将检测结果添加到列表中 [class_id, x, y, z]（机体坐标系）
                detections.append([float(class_id), float(p_B[0][0]), float(p_B[1][0]), float(p_B[2][0])])

                # 添加调试信息
                rospy.logdebug(
                    f"""
                    Detection:
                    - Class ID: {class_id}
                    - Camera frame: {position}
                    - Body frame: {p_B.flatten()}
                """
                )

            # 发布检测结果
            if detections:
                msg = Float32MultiArray()
                msg.layout.dim = [
                    MultiArrayDimension(label="rows", size=len(detections), stride=4),
                    MultiArrayDimension(label="cols", size=4, stride=1),
                ]
                msg.data = [item for sublist in detections for item in sublist]
                self.object_pub.publish(msg)

        except Exception as e:
            LOGGER.error(f"Error processing image: {e}")

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            LOGGER.info("Shutting down")
        cv2.destroyAllWindows()

    def update_camera_rotation(self):
        """更新相机到机体坐标系的旋转矩阵"""
        # 将角度转换为弧度
        pitch = self.camera_pitch * math.pi / 180
        yaw = self.camera_yaw * math.pi / 180
        roll = self.camera_roll * math.pi / 180

        # 分别计算三个轴的旋转矩阵
        Rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, math.cos(pitch), -math.sin(pitch)], [0.0, math.sin(pitch), math.cos(pitch)]]
        )

        Ry = np.array([[math.cos(yaw), 0.0, math.sin(yaw)], [0.0, 1.0, 0.0], [-math.sin(yaw), 0.0, math.cos(yaw)]])

        Rz = np.array([[math.cos(roll), -math.sin(roll), 0.0], [math.sin(roll), math.cos(roll), 0.0], [0.0, 0.0, 1.0]])

        # 基础旋转矩阵（相机朝前时的旋转）
        R_base = np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ]
        )

        # 组合所有旋转：基础旋转 * 横滚 * 俯仰 * 偏航
        self.R_B_C = np.matmul(np.matmul(np.matmul(R_base, Rz), Rx), Ry)


def main():
    try:
        localizer = ObjectLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
