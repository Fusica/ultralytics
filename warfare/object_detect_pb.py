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

        # 相机到机体的变换（相机朝前，与机体系一致）
        self.R_B_C = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        self.t_B_C = np.array([[0.0], [0.0], [0.0]])

        # 固��高度
        self.fixed_height = 0.8  # meters

        # 从相机内参矩阵中提取焦距和光心
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        # 发布器和订阅器
        self.object_pub = rospy.Publisher("/detected_objects", Float32MultiArray, queue_size=10)
        self.coordinates_pub = rospy.Publisher("/coord", Point, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        # self.height_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.height_callback)

    def calculate_3d_position(self, pixel_x, pixel_y, bbox_width, bbox_height):
        """
        根据物体的像素尺寸和实际尺寸计算物体在相机坐标系下的3D位置
        """
        # 已知物体的实际尺寸（单位：米）
        real_height = 0.19  # 19厘米
        real_width = 0.07  # 7厘米

        # 利用相似三角形计算距离
        distance_height = (self.fy * real_height) / bbox_height
        distance_width = (self.fx * real_width) / bbox_width
        distance = (distance_height + distance_width) / 2

        # 计算相机坐标系下的X和Y
        x = ((pixel_x - self.cx) * distance) / self.fx
        y = ((pixel_y - self.cy) * distance) / self.fy
        z = distance

        position = np.array([x, y, z])

        # 发布 Point 消息
        point_msg = Point()
        point_msg.x = float(position[0])
        point_msg.y = float(position[1])
        point_msg.z = float(position[2])
        self.coordinates_pub.publish(point_msg)

        return position

    def height_callback(self, msg):
        """
        处理无人机高度信息
        """
        try:
            # 从 PoseStamped 消息中获取 z 轴位置
            self.fixed_height = msg.pose.position.z

            # 确保高度为正值且在合理范围内
            if self.fixed_height < 0.1:  # 小于 0.1m 认为不合理
                self.fixed_height = 0.8  # 使用默认值
            elif self.fixed_height > 100:  # 大于 100m 认为不合理
                self.fixed_height = 0.8  # 使用默认值

        except Exception as e:
            rospy.logwarn(f"Error processing height: {e}")
            self.fixed_height = 0.8

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

                # 获取边界框中心点（直接使用，不需要映射）
                bbox = det.xyxy[0].cpu().numpy()
                center_x = (bbox[0] + bbox[2]) / 2
                center_y = (bbox[1] + bbox[3]) / 2
                bbox_width = bbox[2] - bbox[0]
                bbox_height = bbox[3] - bbox[1]

                # 添加调试信息来验证坐标
                # rospy.loginfo(f"Detection box coordinates: {bbox}")
                # rospy.loginfo(f"Center point: ({center_x}, {center_y})")

                # 计算3D位置
                position = self.calculate_3d_position(center_x, center_y, bbox_width, bbox_height)

                # 相机坐标系到机体坐标系转换
                p_C = position.reshape(3, 1)
                p_B = self.R_B_C @ p_C + self.t_B_C

                # 添加调试信息
                # rospy.loginfo(f"Body frame position: x={p_B[0][0]:.2f}m, y={p_B[1][0]:.2f}m, z={p_B[2][0]:.2f}m")

                # 将检测结果添加到列表中 [class_id, x, y, z]
                detections.append([float(class_id), float(p_B[0][0]), float(p_B[1][0]), float(p_B[2][0])])

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
