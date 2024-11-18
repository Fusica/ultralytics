#!/usr/bin/env python3

import rosbag
import cv2
from cv_bridge import CvBridge
import os
import argparse
from sensor_msgs.msg import Image


def extract_images(bag_path):
    # 创建输出目录
    output_dir = os.path.join(os.path.dirname(bag_path), "extracted_images")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 初始化 CvBridge
    bridge = CvBridge()

    # 打开rosbag文件
    with rosbag.Bag(bag_path, "r") as bag:
        # 获取图像消息的总数
        total_msgs = bag.get_message_count(topic_filters=["/camera/image_raw"])
        print(f"总共有 {total_msgs} 帧图像需要处理")

        # 计数器
        count = 0

        # 遍历指定topic的所有消息
        for topic, msg, t in bag.read_messages(topics=["/camera/image_raw"]):
            try:
                # 将ROS图像消息转换为OpenCV格式
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # 生成输出文件名（使用时间戳）
                timestamp = t.to_nsec()
                output_path = os.path.join(output_dir, f"frame_{timestamp}.jpg")

                # 保存图像
                cv2.imwrite(output_path, cv_image)

                count += 1
                if count % 100 == 0:
                    print(f"已处理 {count}/{total_msgs} 帧")

            except Exception as e:
                print(f"处理图像时出错: {str(e)}")
                continue

    print(f"处理完成！共保存了 {count} 帧图像到 {output_dir}")


def main():
    # 设置命令行参数
    parser = argparse.ArgumentParser(description="从rosbag中提取图像帧")
    parser.add_argument("bag_path", type=str, help="rosbag文件的路径")
    args = parser.parse_args()

    # 检查文件是否存在
    if not os.path.exists(args.bag_path):
        print(f"错误：找不到rosbag文件 {args.bag_path}")
        return

    # 提取图像
    extract_images(args.bag_path)


if __name__ == "__main__":
    main()
