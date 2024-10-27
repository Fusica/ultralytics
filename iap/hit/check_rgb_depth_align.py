import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def read_images_from_bag(bag_file, color_topic, depth_topic):
    bridge = CvBridge()
    color_images = []
    depth_images = []
    color_timestamps = []
    depth_timestamps = []

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[color_topic, depth_topic]):
            if topic == color_topic:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                color_images.append(cv_image)
                color_timestamps.append(t.to_nsec())
            elif topic == depth_topic:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth_images.append(cv_image)
                depth_timestamps.append(t.to_nsec())

    return color_images, depth_images, color_timestamps, depth_timestamps


def sync_and_visualize(color_images, depth_images, color_timestamps, depth_timestamps):
    color_idx = 0
    depth_idx = 0

    while color_idx < len(color_images) and depth_idx < len(depth_images):
        color_time = color_timestamps[color_idx]
        depth_time = depth_timestamps[depth_idx]

        if abs(color_time - depth_time) < 1e7:  # 10ms threshold
            color_img = color_images[color_idx]
            depth_img = depth_images[depth_idx]

            # Normalize depth image for visualization
            depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_colormap = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

            # Resize depth image to match color image size
            depth_resized = cv2.resize(depth_colormap, (color_img.shape[1], color_img.shape[0]))

            # Blend color and depth images
            alpha = 0.7
            blended = cv2.addWeighted(color_img, alpha, depth_resized, 1 - alpha, 0)

            cv2.imshow("Synchronized Color and Depth", blended)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            color_idx += 1
            depth_idx += 1
        elif color_time < depth_time:
            color_idx += 1
        else:
            depth_idx += 1

    cv2.destroyAllWindows()


def main():
    bag_file = "/home/ubuntu/hit_bags/zed_data_d435i.bag"
    color_topic = "/camera/color/image_raw"
    depth_topic = "/camera/depth/image_rect_raw"

    color_images, depth_images, color_timestamps, depth_timestamps = read_images_from_bag(
        bag_file, color_topic, depth_topic
    )
    sync_and_visualize(color_images, depth_images, color_timestamps, depth_timestamps)


if __name__ == "__main__":
    main()
