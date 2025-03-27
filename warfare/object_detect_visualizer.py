#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import logging

LOGGER = logging.getLogger(__name__)

class ObjectDetectionVisualizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("object_detection_visualizer")
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Window name
        self.window_name = "Object Detection Visualization"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Class names (modify as needed for your specific model)
        self.class_names = {0: "Soldier"}  # Add more classes as needed
        
        # Colors for different classes (BGR format)
        self.colors = {0: (0, 255, 0)}  # Green for soldiers, add more as needed
        
        # Latest received data
        self.latest_image = None
        self.latest_detections = None
        self.image_timestamp = None
        self.detections_timestamp = None
        
        # Max time difference for message synchronization (in seconds)
        self.max_sync_diff = rospy.get_param("~max_sync_diff", 0.3)
        
        # Display settings
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.text_thickness = 1
        self.box_thickness = 2
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            "/detection_image/compressed", 
            CompressedImage, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.detections_sub = rospy.Subscriber(
            "/detection_boxes", 
            Float32MultiArray, 
            self.detections_callback,
            queue_size=10
        )
        
        # Timer for display updates
        self.display_timer = rospy.Timer(rospy.Duration(0.05), self.display_callback)  # 20 FPS
        
        rospy.loginfo("Object Detection Visualizer initialized")

    def image_callback(self, msg):
        """Process received compressed images"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image_timestamp = msg.header.stamp
        except Exception as e:
            LOGGER.error(f"Error processing compressed image: {e}")

    def detections_callback(self, msg):
        """Process received detection messages"""
        try:
            if len(msg.data) >= 6:
                # 从检测框数据中提取时间戳
                # 第一行为 [timestamp, 0, 0, 0, 0, 0]
                timestamp_secs = msg.data[0]
                self.detections_timestamp = rospy.Time(timestamp_secs)
                
                # 保存实际的检测数据（跳过时间戳行）
                self.latest_detections = msg.data[6:]
            else:
                rospy.logwarn("Received detection message with insufficient data")
                self.latest_detections = None
                self.detections_timestamp = None
                
        except Exception as e:
            LOGGER.error(f"Error processing detections: {e}")
            self.latest_detections = None
            self.detections_timestamp = None

    def display_callback(self, event):
        """Update the display with latest image and detections"""
        if self.latest_image is None:
            return
            
        # Create a copy of the image to draw on
        display_image = self.latest_image.copy()
        
        # If we have detections that are close in time to the image
        if (self.latest_detections is not None and self.image_timestamp is not None and 
            self.detections_timestamp is not None):
            
            # Check if timestamps are close enough
            time_diff = abs((self.image_timestamp - self.detections_timestamp).to_sec())
            
            if time_diff <= self.max_sync_diff:
                # Add timestamp synchronization info
                cv2.putText(
                    display_image,
                    f"Sync diff: {time_diff*1000:.1f}ms",
                    (10, 30),
                    self.font,
                    0.7,
                    (0, 255, 0),
                    1
                )
                
                # Draw detections
                # 数据格式: [class_id, x1, y1, x2, y2, conf, class_id, x1, y1, x2, y2, conf, ...]
                for i in range(0, len(self.latest_detections), 6):
                    try:
                        # Get detection data
                        class_id = int(self.latest_detections[i])
                        xmin = int(self.latest_detections[i+1])
                        ymin = int(self.latest_detections[i+2])
                        xmax = int(self.latest_detections[i+3])
                        ymax = int(self.latest_detections[i+4])
                        confidence = self.latest_detections[i+5]
                        
                        # Get color for this class
                        color = self.colors.get(class_id, (255, 0, 0))  # Default to blue if class not in colors
                        
                        # Draw bounding box
                        cv2.rectangle(display_image, (xmin, ymin), (xmax, ymax), color, self.box_thickness)
                        
                        # Draw class name and confidence
                        class_name = self.class_names.get(class_id, f"Class {class_id}")
                        label = f"{class_name}: {confidence:.2f}"
                        
                        # Get label size for background rectangle
                        (label_width, label_height), baseline = cv2.getTextSize(
                            label, self.font, self.font_scale, self.text_thickness
                        )
                        
                        # Draw label background
                        cv2.rectangle(
                            display_image, 
                            (xmin, ymin - label_height - 5), 
                            (xmin + label_width, ymin), 
                            color, 
                            -1  # filled
                        )
                        
                        # Draw label text
                        cv2.putText(
                            display_image, 
                            label, 
                            (xmin, ymin - 5), 
                            self.font, 
                            self.font_scale, 
                            (0, 0, 0),  # Black text
                            self.text_thickness
                        )
                    except Exception as e:
                        LOGGER.error(f"Error drawing detection: {e}")
            else:
                # Draw message about synchronization
                cv2.putText(
                    display_image,
                    f"Message sync error: {time_diff:.2f}s",
                    (10, 30),
                    self.font,
                    1.0,
                    (0, 0, 255),
                    2
                )
                
                # Draw timestamps for debugging
                cv2.putText(
                    display_image,
                    f"Img time: {self.image_timestamp.to_sec():.2f}, Det time: {self.detections_timestamp.to_sec():.2f}",
                    (10, 60),
                    self.font,
                    0.7,
                    (0, 0, 255),
                    1
                )
                
        
        # Add FPS and timestamp info
        current_time = rospy.Time.now()
        cv2.putText(
            display_image,
            f"Time: {current_time.to_sec():.2f}",
            (10, display_image.shape[0] - 10),
            self.font,
            0.5,
            (255, 255, 255),
            1
        )
        
        # Show the image
        cv2.imshow(self.window_name, display_image)
        cv2.waitKey(1)

    def run(self):
        """Run the node"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            LOGGER.info("Shutting down")
        cv2.destroyAllWindows()

def main():
    try:
        visualizer = ObjectDetectionVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main() 