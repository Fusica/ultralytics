#!/usr/bin/env python3
import sys
import gi
import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Optional
import signal

# 加载 GStreamer 的库
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


class GstreamerVideoStreamer:
    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 1920,
        height: int = 1080,
        framerate: int = 60,
        raw_topic: str = "/camera/image_raw",
        compressed_topic: str = "/camera/image_raw/compressed",
        compress_quality: int = 10,
    ):
        """初始化视频流处理器

        Args:
            device: 摄像头设备路径
            width: 视频宽度
            height: 视频高度
            framerate: 帧率
            raw_topic: 原始图像ROS话题名称
            compressed_topic: 压缩图像ROS话题名称
            compress_quality: JPEG压缩质量(1-100)
        """
        # 初始化 GStreamer
        Gst.init(None)

        # 初始化 ROS
        rospy.init_node("gstreamer_video_streamer", anonymous=True)
        self.raw_pub = rospy.Publisher(raw_topic, Image, queue_size=10)
        self.compressed_pub = rospy.Publisher(compressed_topic, CompressedImage, queue_size=10)
        self.bridge = CvBridge()

        # 创建pipeline
        self.pipeline = None
        self.loop = None
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.compress_quality = compress_quality

    def _create_pipeline(self) -> str:
        """创建 GStreamer pipeline 字符串"""
        return (
            f"v4l2src device={self.device} ! "
            f"image/jpeg,width={self.width},height={self.height},"
            f"framerate={self.framerate}/1 ! "
            "jpegdec ! "
            "nvvidconv ! "
            "video/x-raw,format=I420 ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )

    def _compress_frame(self, frame: np.ndarray) -> bytes:
        """压缩图像帧

        Args:
            frame: BGR格式的图像帧

        Returns:
            压缩后的JPEG字节数据
        """
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compress_quality]
        result, encimg = cv2.imencode(".jpg", frame, encode_param)
        if not result:
            raise Exception("Failed to compress image")
        return encimg.tobytes()

    def on_frame(self, sink, data) -> Gst.FlowReturn:
        """处理每一帧的回调函数"""
        try:
            sample = sink.emit("pull-sample")
            if not sample:
                return Gst.FlowReturn.ERROR

            buf = sample.get_buffer()
            caps = sample.get_caps()

            # 获取帧的数据
            height = caps.get_structure(0).get_value("height")
            width = caps.get_structure(0).get_value("width")
            array = np.ndarray((height * 3 // 2, width), buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)

            # 转换颜色空间
            frame = cv2.cvtColor(array, cv2.COLOR_YUV2BGR_I420)

            # 发布原始图像
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.raw_pub.publish(raw_msg)

            # 发布压缩图像
            compressed_msg = CompressedImage()
            compressed_msg.header = raw_msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = self._compress_frame(frame)
            self.compressed_pub.publish(compressed_msg)

            return Gst.FlowReturn.OK

        except Exception as e:
            rospy.logerr(f"Error processing frame: {str(e)}")
            return Gst.FlowReturn.ERROR

    def signal_handler(self, sig, frame):
        """处理终止信号"""
        rospy.loginfo("\nShutting down...")
        if self.loop:
            self.loop.quit()
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        sys.exit(0)

    def start(self):
        """启动视频流处理"""
        try:
            pipeline_str = self._create_pipeline()
            rospy.loginfo(f"Starting pipeline: {pipeline_str}")

            self.pipeline = Gst.parse_launch(pipeline_str)
            appsink = self.pipeline.get_by_name("sink")
            appsink.connect("new-sample", self.on_frame, None)

            # 设置pipeline状态
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("Unable to set the pipeline to playing state")

            # 设置信号处理
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)

            # 运行主循环
            self.loop = GLib.MainLoop()
            self.loop.run()

        except Exception as e:
            rospy.logerr(f"Error in pipeline: {str(e)}")
            self.signal_handler(None, None)


def main():
    try:
        streamer = GstreamerVideoStreamer()
        streamer.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
