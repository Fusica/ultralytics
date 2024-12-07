#!/usr/bin/env python3
import cv2
import numpy as np
from threading import Thread
import time
import logging
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RTSPtoROSPublisher:
    """
    将RTSP视频流转换为ROS图像消息并发布
    
    Attributes:
        rtsp_url (str): RTSP流地址
        topic_name (str): ROS发布主题名
        frame_id (str): ROS消息帧ID
        publish_rate (float): 发布频率（Hz）
    """
    
    def __init__(self, rtsp_url, topic_name="/camera/image_raw", frame_id="camera", publish_rate=30.0):
        # 初始化ROS节点
        rospy.init_node('rtsp_publisher', anonymous=True)
        
        # 创建ROS发布者和CV桥接器
        self.publisher = rospy.Publisher(topic_name, Image, queue_size=1)
        self.bridge = CvBridge()
        self.frame_id = frame_id
        self.publish_rate = publish_rate
        
        # 初始化RTSP加载器
        self.loader = RTSPLoader(rtsp_url)
        
        rospy.loginfo(f"Initialized RTSP publisher for {rtsp_url}")
        rospy.loginfo(f"Publishing to topic: {topic_name}")
        
    def publish_frames(self):
        """持续发布视频帧"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            try:
                # 读取视频帧
                frame = self.loader.read()
                if frame is None:
                    rospy.logwarn("Failed to read frame")
                    continue
                
                # 转换为ROS图像消息
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id
                
                # 发布消息
                self.publisher.publish(msg)
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in publish_frames: {e}")
    
    def run(self):
        """运行发布器"""
        try:
            rospy.loginfo("Starting frame publication...")
            self.publish_frames()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        finally:
            self.close()
    
    def close(self):
        """清理资源"""
        self.loader.close()
        rospy.loginfo("RTSP publisher closed")

class RTSPLoader:
    """
    RTSP视频流加载器
    
    Attributes:
        source (str): RTSP流地址
        buffer (bool): 是否缓存图像帧
        fps (float): 视频帧率
        shape (tuple): 图像形状 (height, width, channels)
    """
    
    def __init__(self, rtsp_url, buffer=False):
        """初始化RTSP加载器"""
        self.source = rtsp_url
        self.buffer = buffer
        self.running = True
        
        # 初始化视频捕获
        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            raise ConnectionError(f"Failed to open RTSP stream: {rtsp_url}")
            
        # 获取视频信息
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = max(self.cap.get(cv2.CAP_PROP_FPS) % 100, 30)  # 使用30fps作为后备值
        
        # 读取第一帧以确保连接正常
        success, first_frame = self.cap.read()
        if not success or first_frame is None:
            raise ConnectionError(f"Failed to read frame from: {rtsp_url}")
            
        # 初始化图像缓冲区
        self.imgs = []
        self.imgs.append(first_frame)
        self.shape = first_frame.shape
        
        # 启动读取线程
        self.thread = Thread(target=self.update, daemon=True)
        self.thread.start()
        
        rospy.loginfo(f"Successfully connected to RTSP stream: {self.width}x{self.height} at {self.fps:.2f} FPS")
    
    def update(self):
        """在后台线程中持续读取视频帧"""
        while self.running and self.cap.isOpened():
            if len(self.imgs) < 30:  # 保持不超过30帧的缓冲
                success, frame = self.cap.read()
                if success:
                    if self.buffer:
                        self.imgs.append(frame)
                    else:
                        self.imgs = [frame]  # 只保留最新帧
                else:
                    rospy.logwarn("Video stream unresponsive, attempting to reconnect...")
                    self.cap.open(self.source)  # 尝试重新连接
            else:
                time.sleep(0.01)  # 等待缓冲区清空
                
    def read(self):
        """读取当前帧"""
        while not self.imgs:
            if not self.thread.is_alive():
                return None
            time.sleep(1/self.fps)
            
        if self.buffer:
            return self.imgs.pop(0)  # 返回最早的帧
        else:
            frame = self.imgs.pop(-1)  # 返回最新的帧
            self.imgs.clear()
            return frame
    
    def close(self):
        """关闭视频流和释放资源"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=5)
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        # 创建并运行发布器
        rtsp_url = "rtsp://192.168.1.25:8554/main.264"
        publisher = RTSPtoROSPublisher(
            rtsp_url=rtsp_url,
            topic_name="/camera/image_raw",
            frame_id="camera",
            publish_rate=30.0
        )
        publisher.run()
    except rospy.ROSInterruptException:
        pass
