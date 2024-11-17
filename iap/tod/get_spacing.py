from ultralytics import YOLO
import numpy as np

import pyrealsense2 as rs
import rospy
from std_msgs.msg import Float32

actual_tod_height = 1.56

# ros node init
rospy.init_node('tod_width_pub')
width_pub = rospy.Publisher('/tod_width', Float32, queue_size=10)

model = YOLO("/home/robot/ultralytics/runs/detect/train102/weights/best.pt")

# realtime image capture
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

last_tod_width = None  # Store the last detected width

try:
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            if last_tod_width is not None:
                # Publish the last known width if no new frame is available
                width_pub.publish(last_tod_width)
            continue

        color_image = np.asarray(color_frame.get_data())

        results = model(color_image)

        boxes = results[0].boxes

        tod_boxes = [box for box in boxes if box.cls == 0]
        tod_boxes = sorted(tod_boxes, key=lambda x: x.conf, reverse=True)

        if len(tod_boxes) < 2:
            print("No enough tod detected!")
            if last_tod_width is not None:
                # Publish the last known width if no new detection is available
                width_pub.publish(last_tod_width)
        else:
            selected_boxes = []
            for i in range(len(tod_boxes)):
                for j in range(i+1, len(tod_boxes)):
                    box1 = tod_boxes[i]
                    box2 = tod_boxes[j]
                    box1_xyxy = box1.xyxy.cpu().numpy().flatten()
                    box2_xyxy = box2.xyxy.cpu().numpy().flatten()
                    center1_x = (box1_xyxy[0] + box1_xyxy[3]) / 2
                    center2_x = (box2_xyxy[0] + box2_xyxy[3]) / 2

                    if abs(center1_x - center2_x) > 25:
                        selected_boxes = [box1, box2]
                        break
                if selected_boxes:
                    break

            if selected_boxes:
                box1, box2 = selected_boxes
                box1_xyxy = box1.xyxy.cpu().numpy().flatten()
                box2_xyxy = box2.xyxy.cpu().numpy().flatten()

                height1 = box1_xyxy[3] - box1_xyxy[1]
                height2 = box2_xyxy[3] - box2_xyxy[1]

                height = (height1 + height2) / 2

                box1_center = np.array([(box1_xyxy[0] + box1_xyxy[2]) / 2, (box1_xyxy[1] + box1_xyxy[3]) / 2])
                box2_center = np.array([(box2_xyxy[0] + box2_xyxy[2]) / 2, (box2_xyxy[1] + box2_xyxy[3]) / 2])

                tod_width = np.linalg.norm(box2_center - box1_center)

                actual_tod_width = actual_tod_height * tod_width / height

                last_tod_width = actual_tod_width  # Update the last detected width

                width_pub.publish(actual_tod_width)

finally:
    pipeline.stop()