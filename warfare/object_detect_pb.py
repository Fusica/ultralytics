#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
from ultralytics import YOLO


class ObjectLocalizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.yolo = YOLO("/path/to/your/weights.pt")

        # 相机内参
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

        # 订阅器
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # 发布器
        self.object_pose_pub = rospy.Publisher("/detected_objects_world", PoseStamped, queue_size=10)

    def drone_pose_callback(self, msg):
        self.current_drone_pose = msg

    def estimate_distance(self, pixel_y, fov_vertical=60):
        """估算目标距离
        Args:
            pixel_y: 目标在图像中的y坐标
            fov_vertical: 相机垂直视场角(度)
        Returns:
            估算的距离(米)
        """
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

    def image_callback(self, msg):
        if self.current_drone_pose is None:
            return

        # 处理图像
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.yolo(image)[0]

        for det in results.boxes:
            if det.conf < 0.5:
                continue

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

            # 发布结果（机体坐标系）
            pose_msg = PoseStamped()
            pose_msg.header = self.current_drone_pose.header
            pose_msg.pose.position.x = float(p_B[0])
            pose_msg.pose.position.y = float(p_B[1])
            pose_msg.pose.position.z = float(p_B[2])
            self.object_pose_pub.publish(pose_msg)


def main():
    rospy.init_node("object_localizer")
    localizer = ObjectLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
