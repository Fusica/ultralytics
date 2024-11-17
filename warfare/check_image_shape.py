import rosbag
from sensor_msgs.msg import Image

# 打开 Bag 文件
bag = rosbag.Bag("/home/max/ultralytics/warfare/2024-11-16-23-56-58.bag")

# 遍历话题
for topic, msg, t in bag.read_messages(topics=["/camera/image_raw"]):
    print(f"Width: {msg.width}, Height: {msg.height}")
    break  # 只查看第一帧即可
bag.close()
