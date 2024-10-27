from ultralytics import YOLO
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# ROS node init
rospy.init_node('detect')
width_pub = rospy.Publisher('/detect_object', Float32, queue_size=10)

# YOLO model loading
model = YOLO("yolo11n.pt")

# Bridge to convert ROS Image messages to OpenCV images
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        color_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return

    # Run YOLO model on the image
    results = model.predict(color_image, imgsz=2160)

    # Extract the detected boxes
    boxes = results[0].boxes

    # Check if any boxes are detected
    if len(boxes) > 0:
        # Assuming you want to publish the width of the first detected box
        box = boxes[0]
        box_xyxy = box.xyxy.cpu().numpy().flatten()
        width = box_xyxy[2] - box_xyxy[0]
        width_pub.publish(Float32(width))
    else:
        rospy.loginfo("No objects detected!")

# Subscribe to the image topic
image_topic = "/camera/image_raw"  # Modify this topic as per your setup
rospy.Subscriber(image_topic, Image, image_callback)

# Keep the node running
rospy.spin()