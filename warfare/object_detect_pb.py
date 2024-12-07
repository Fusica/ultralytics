#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float64MultiArray, Float32MultiArray
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

        # TODO 相机到机体坐标系变换矩阵（相机朝前，与机体系一致），需要每次校准外参
        self.t_B_C = np.array([[0], [0], [0]])

        # TODO 测试使用，真实环境需要去掉直接调用local_position/pose
        self.height = 0.66

        # 发布器和订阅器
        self.object_pub = rospy.Publisher("/detected_objects", Float32MultiArray, queue_size=10)
        self.coordinates_pub = rospy.Publisher("/camera_coordinates", Point, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        self.height_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.height_callback)

        # 相机姿态 (rad)
        self.cam_roll = 0.0
        self.cam_pitch = 0.0
        self.cam_yaw = 0.0

        # 订阅云台姿态
        self.rpy_sub = rospy.Subscriber("/siyi/rpy", Float64MultiArray, self.rpy_callback)

    def undistort_pixel(self, pixel_x, pixel_y):
        # 相机内参矩阵
        K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])

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

    def calculate_camera_coordinates(self, pixel_x, pixel_y):
        """
        根据物体的像素坐标、相机内参、云台姿态和无人机高度计算物体在相机坐标系下的三维坐标。
        相机坐标系：X右、Y下、Z前
        """
        h = self.height  # 无人机高度（向下为正）

        # 去畸变后的像素坐标
        undistorted_x, undistorted_y = self.undistort_pixel(pixel_x, pixel_y)

        # 相机坐标系归一化方向
        x_c = (undistorted_x - self.cx) / self.fx
        y_c = (undistorted_y - self.cy) / self.fy
        z_c = 1.0
        ray_dir = np.array([x_c, y_c, z_c], dtype=np.float64)
        ray_dir = ray_dir / np.linalg.norm(ray_dir)

        # 地面在机体坐标系下的法向量和方程
        n_b = np.array([0, 0, 1], dtype=np.float64)  # 地面法向量(机体坐标)
        # 平面方程: n_b · P_b = h (地面在Z=h处)

        # 将法向量转换到相机坐标系
        # R_B_C 已知，R_C_B = R_B_C^T
        R_C_B = self.R_B_C.T
        n_c = R_C_B @ n_b  # 相机坐标系下的地面法向量

        # 地面方程在相机坐标下： n_c · P_c = h
        # 光线求交： n_c · (t * ray_dir) = h => t = h / (n_c · ray_dir)

        denom = n_c.dot(ray_dir)
        if abs(denom) < 1e-6:
            rospy.logwarn("Ray is nearly parallel to ground, cannot calculate intersection")
            return None

        t = h / denom
        if t < 0:
            rospy.logwarn("Intersection point is behind the camera")
            return None

        camera_coordinates = t * ray_dir
        return camera_coordinates

    def transform_camera_to_body(self, camera_coordinates):
        """
        将相机坐标系下的位置转换到机体坐标系
        Args:
            position_camera: numpy.ndarray, 相机坐标系下的位置 [x右, y下, z前]
        Returns:
            numpy.ndarray: 机体坐标系下的位置 [x前, y右, z上]
        """
        try:
            # 使用当前云台姿态的旋转矩阵进行转换
            position_body = self.R_B_C @ camera_coordinates + self.t_B_C.flatten()
            return position_body
        except Exception as e:
            rospy.logerr(f"Error in coordinate transformation: {e}")
            return None

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

    def rpy_callback(self, msg):
        """处理云台姿态数据"""
        self.cam_roll = math.radians(msg.data[0])  # rad
        self.cam_pitch = math.radians(msg.data[1])  # rad
        self.cam_yaw = math.radians(msg.data[2])  # rad

        cr = math.cos(self.cam_roll)
        sr = math.sin(self.cam_roll)
        cp = math.cos(self.cam_pitch)
        sp = math.sin(self.cam_pitch)
        cy = math.cos(self.cam_yaw)
        sy = math.sin(self.cam_yaw)

        R_base = np.array(
            [
                [0.0, 0.0, 1.0],  # 相机Z轴对应机体X轴
                [1.0, 0.0, 0.0],  # 相机X轴对应机体Y轴
                [0.0, 1.0, 0.0],  # 相机Y轴对应机体Z轴
            ]
        )

        # 分别构造三个基本旋转矩阵
        Rz_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        Rx_pitch = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])

        Ry_roll = np.array([[cr, 0, sr], [0, 1, 0], [-sr, 0, cr]])

        # 最终旋转矩阵
        R_rpy = Rz_yaw @ Rx_pitch @ Ry_roll

        # 将基本对齐矩阵R_base乘上R_rpy得到R_B_C
        self.R_B_C = R_rpy @ R_base

    def image_callback(self, msg):
        """处理接收到的图像消息"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.yolo(frame, imgsz=1280)[0]
            detections = []

            for det in results.boxes:
                if det.conf < 0.6:
                    continue

                class_id = int(det.cls[0])
                bbox = det.xyxy[0].cpu().numpy()
                center_x = (bbox[0] + bbox[2]) / 2
                center_y = (bbox[1] + bbox[3]) / 2
                bottom_y = bbox[3]

                # 计算相机坐标系下的位置
                camera_coordinates = self.calculate_camera_coordinates(center_x, bottom_y)
                # 发布相机坐标系下的方向向量（归一化后的）
                camera_point = Point()
                camera_point.x = float(camera_coordinates[0])
                camera_point.y = float(camera_coordinates[1])
                camera_point.z = float(camera_coordinates[2])
                self.coordinates_pub.publish(camera_point)

                # 转换到机体坐标系
                body_coordinates = self.transform_camera_to_body(camera_coordinates)

                detections.append(
                    [
                        float(class_id),
                        float(body_coordinates[0]),
                        float(body_coordinates[1]),
                        float(body_coordinates[2]),
                    ]
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


def main():
    try:
        localizer = ObjectLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
