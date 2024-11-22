#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import logging
from geometry_msgs.msg import PoseStamped, Point
import math
from tf.transformations import quaternion_matrix, euler_from_quaternion

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

        # 提取相机内参
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        # TODO 相机到机体的变换（相机朝前，与机体系一致），需要每次校准外参
        self.R_B_C = np.eye(3)
        self.t_B_C = np.zeros((3, 1))

        # 初始化无人机高度
        self.height = 0.8

        # 添加当前无人机位姿
        self.current_drone_pose = None

        # 发布器和订阅器
        self.object_pub = rospy.Publisher("/detected_objects", Float32MultiArray, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        self.height_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.height_callback)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_callback)

    def undistort_pixel(self, pixel_x, pixel_y):
        """
        去畸变像素坐标
        """
        # 相机内参矩阵
        K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])

        # 畸变系数
        dist_coeffs = np.array([self.k1, self.k2, self.p1, self.p2])

        # 像素坐标
        pts = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)

        # 去畸变
        undistorted_pts = cv2.undistortPoints(pts, K, dist_coeffs, P=K)

        # 提取去畸变后的像素坐标
        undistorted_x = undistorted_pts[0][0][0]
        undistorted_y = undistorted_pts[0][0][1]

        return undistorted_x, undistorted_y

    def calculate_3d_position(self, pixel_x, pixel_y):
        """
        根据物体中心的像素坐标、相机内参和无人机高度计算物体在机体坐标系下的三维坐标。
        """
        # 获取无人机高度（相机高度）
        h = self.height

        # 去畸变像素坐标
        undistorted_x, undistorted_y = self.undistort_pixel(pixel_x, pixel_y)

        # 计算相对于光心的像素偏移量
        delta_x = undistorted_x - self.cx
        delta_y = undistorted_y - self.cy

        # 计算偏移角度（弧度）
        theta_x = math.atan(delta_x / self.fx)
        theta_y = math.atan(delta_y / self.fy)

        # 计算相机坐标系下的 x 和 y（物体位于地平面，z=0）
        x = h * math.tan(theta_x)
        y = h * math.tan(theta_y)
        z = 0  # 物体位于地平面

        position = np.array([x, y, z])
        return position

    def height_callback(self, msg):
        """
        处理无人机高度信息
        """
        try:
            self.height = msg.pose.position.z
            rospy.loginfo(f"Current UAV Height: {self.height} meters")
        except Exception as e:
            rospy.logwarn(f"Error processing height: {e}")

    def drone_pose_callback(self, msg):
        """
        处理无人机位姿数据
        """
        try:
            self.current_drone_pose = msg
            rospy.logdebug(f"Drone position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")
        except Exception as e:
            rospy.logwarn(f"Error processing drone pose: {e}")

    def transform_to_world_frame(self, position_body):
        """
        将机体坐标系下的位置转换到世界坐标系
        """
        if self.current_drone_pose is None:
            rospy.logwarn("No drone pose available for coordinate transformation")
            return position_body

        try:
            # 获取无人机在世界系下的位置
            drone_pos = np.array(
                [
                    [self.current_drone_pose.pose.position.x],
                    [self.current_drone_pose.pose.position.y],
                    [self.current_drone_pose.pose.position.z],
                ]
            )

            # 获取无人机姿态四元数
            q = self.current_drone_pose.pose.orientation
            # 将四元数转换为旋转矩阵（世界系到机体系的转换）
            R_W_B = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

            # 将位置转换为列向量
            p_B = np.array(position_body).reshape(3, 1)

            # 转换到世界坐标系: p_W = R_W_B @ p_B + drone_pos
            p_W = R_W_B @ p_B + drone_pos

            return p_W.flatten()

        except Exception as e:
            rospy.logerr(f"Error in coordinate transformation: {e}")
            return position_body

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

                # 转换到世界坐标系
                p_W = self.transform_to_world_frame(p_B.flatten())

                # 将检测结果添加到列表中 [class_id, x, y, z]（世界坐标系）
                detections.append([float(class_id), float(p_W[0]), float(p_W[1]), float(p_W[2])])

                # 添加调试信息
                rospy.logdebug(
                    f"""
                    Detection:
                    - Class ID: {class_id}
                    - Camera frame: {position}
                    - Body frame: {p_B.flatten()}
                    - World frame: {p_W}
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


def main():
    try:
        localizer = ObjectLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
