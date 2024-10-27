#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO
from cv_bridge import CvBridge

R_B_C = [
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
]

t_B_C = np.array([[0.36], [0.0], [0.0]])

# 初始化YOLO模型
detection_model = YOLO("/home/robot/ultralytics/runs/detect/train83/weights/best.pt")
bridge = CvBridge()

# 初始化ROS节点
rospy.init_node("ultralytics_yolo")

# 创建发布者
point_pub = rospy.Publisher("/center_zed", Point, queue_size=5)

# 全局变量存储最新的深度图像
latest_depth_image = None

# 置信度阈值
CONFIDENCE_THRESHOLD = 0.7


def depth_callback(depth_msg):
    global latest_depth_image
    latest_depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")


import numpy as np


def calculate_distance(depth_image, bbox):
    x1, y1, x2, y2 = map(int, bbox)

    # 计算边界框的中心点
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2

    # 计算新的边界框尺寸（原来的一半）
    new_width = (x2 - x1) // 2
    new_height = (y2 - y1) // 2

    # 计算新的边界框坐标
    new_x1 = max(center_x - new_width // 2, 0)
    new_y1 = max(center_y - new_height // 2, 0)
    new_x2 = min(center_x + new_width // 2, depth_image.shape[1])
    new_y2 = min(center_y + new_height // 2, depth_image.shape[0])

    # 提取新的ROI
    roi = depth_image[new_y1:new_y2, new_x1:new_x2]
    valid_depths = roi[np.isfinite(roi) & (roi != 0)]

    if len(valid_depths) == 0:
        return None

    avg_depth = np.mean(valid_depths) / 1000.0
    return avg_depth


def process_frame(msg):
    global latest_depth_image

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    det_result = detection_model(frame)
    detections = det_result[0].boxes

    if len(detections) > 0 and latest_depth_image is not None:
        confidences = detections.conf.cpu().numpy()

        # 应用置信度阈值
        high_conf_indices = np.where(confidences > CONFIDENCE_THRESHOLD)[0]

        if len(high_conf_indices) > 0:
            max_conf_index = high_conf_indices[np.argmax(confidences[high_conf_indices])]
            best_det = detections[max_conf_index]
            bbox = best_det.xyxy[0].cpu().numpy()
            cls = best_det.cls[0].cpu().numpy()

            center_x = (bbox[0] + bbox[2]) / 2
            center_y = (bbox[1] + bbox[3]) / 2
            distance = calculate_distance(latest_depth_image, bbox)

            if distance is not None:
                x = (center_x - 325.89190673828125) / 605.593505859375 * distance
                y = (center_y - 245.63575744628906) / 605.5718994140625 * distance

                camera_aixs = np.array([[x], [y], [distance]])
                converted = R_B_C @ camera_aixs + t_B_C
                point_msg = Point(converted[0][0], converted[1][0], converted[2][0])

                # rospy.loginfo(f"x is: {converted[0]}, y is: {converted[1]}, z is: {converted[2]}")

                point_pub.publish(point_msg)

                # # 绘制检测框和信息
                # cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
                # label = f"{detection_model.names[int(cls)]}: {confidences[max_conf_index]:.2f}"
                # cv2.putText(frame, label, (int(bbox[0]), int(bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                # # 显示转换后的坐标
                # coord_text = f"X: {converted[0][0]:.2f}, Y: {converted[1][0]:.2f}, Z: {converted[2][0]:.2f}"
                # cv2.putText(frame, coord_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                rospy.loginfo("Distance calculation failed")
        else:
            rospy.loginfo("No detection above confidence threshold")
    else:
        rospy.loginfo("No detection in this frame or depth image not available")

    # # 显示图像
    # cv2.imshow("YOLO Detection", frame)
    # cv2.waitKey(1)


def main():
    rospy.Subscriber("/camera/color/image_raw", Image, process_frame)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
