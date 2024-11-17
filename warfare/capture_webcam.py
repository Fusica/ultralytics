#!/usr/bin/env python3
import sys
import gi
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

# 加载 GStreamer 的库
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# 初始化 GStreamer
Gst.init(None)

# 创建 ROS 节点
rospy.init_node('gstreamer_video_streamer', anonymous=True)
image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
bridge = CvBridge()

def on_frame(sink, data):
    # 从 GStreamer 中获取原始帧
    sample = sink.emit("pull-sample")
    buf = sample.get_buffer()
    caps = sample.get_caps()
    
    # 获取帧的数据大小
    array = np.ndarray(
        (caps.get_structure(0).get_value("height") * 3 // 2,
         caps.get_structure(0).get_value("width")),
        buffer=buf.extract_dup(0, buf.get_size()),
        dtype=np.uint8)

    # 将 I420 格式转换为 BGR
    frame = cv2.cvtColor(array, cv2.COLOR_YUV2BGR_I420)

    # 将帧数据转换为 ROS Image 消息并发布
    image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    image_pub.publish(image_message)

    return Gst.FlowReturn.OK

def start_pipeline():
    # GStreamer 管道字符串
    pipeline_str = (
    'v4l2src device=/dev/video0 ! '
    'image/jpeg,width=384,height=288,framerate=50/1 ! '
    'jpegdec ! '
    'nvvidconv ! '
    'video/x-raw,format=I420 ! '
    'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
)

    print(pipeline_str)
    # 创建 GStreamer 管道
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name('sink')

    # 连接帧数据的回调函数
    appsink.connect('new-sample', on_frame, None)
    
    # 启动管道
    pipeline.set_state(Gst.State.PLAYING)

    # 循环处理
    try:
        loop = GLib.MainLoop()
        loop.run()
    except:
        pass
    finally:
        # 清理
        pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    try:
        start_pipeline()
    except rospy.ROSInterruptException:
        pass
