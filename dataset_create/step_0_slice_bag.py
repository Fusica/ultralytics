#!/usr/bin/env python3

import rosbag
import cv2
from cv_bridge import CvBridge
import os
import argparse
from sensor_msgs.msg import Image


def extract_images(bag_path, output_dir):
    # 创建输出目录
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
            print(f"Original encoding: {msg.encoding}")  # 查看原始编码
            print(f"Image size: {msg.width}x{msg.height}")
            print(f"Raw data size: {len(msg.data)} bytes")
            try:
                # 保持原始编码格式转换
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)

                # 根据原始编码选择保存格式
                if msg.encoding in ['rgb8', 'bgr8']:
                    output_path = os.path.join(output_dir, f"frame_{t.to_nsec()}.png")
                    cv2.imwrite(output_path, cv_image)
                elif msg.encoding in ['bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8']:
                    # 对于原始Bayer格式，保存为16位TIFF以保持原始数据
                    output_path = os.path.join(output_dir, f"frame_{t.to_nsec()}.tiff")
                    cv2.imwrite(output_path, cv_image, [cv2.IMWRITE_TIFF_COMPRESSION, 1])
                else:
                    print(f"未处理的编码格式: {msg.encoding}")

                count += 1
                if count % 100 == 0:
                    print(f"已处理 {count}/{total_msgs} 帧")

            except Exception as e:
                print(f"处理图像时出错: {str(e)}")
                continue

            # 直接保存原始数据
            with open(os.path.join(output_dir, f"frame_{t.to_nsec()}.raw"), 'wb') as f:
                f.write(msg.data)

    print(f"处理完成！共保存了 {count} 帧图像到 {output_dir}")


def main():
    # 设置命令行参数
    parser = argparse.ArgumentParser(description="从rosbag中提取图像帧")
    parser.add_argument("bag_path", type=str, help="rosbag文件的路径")
    parser.add_argument("output_dir", type=str, help="输出图像的目录")
    args = parser.parse_args()

    # 检查文件是否存在
    if not os.path.exists(args.bag_path):
        print(f"错误：找不到rosbag文件 {args.bag_path}")
        return

    # 提取图像
    extract_images(args.bag_path, args.output_dir)


if __name__ == "__main__":
    main()
