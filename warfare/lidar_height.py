#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class LaserToPoseConverter:
    def __init__(self):
        rospy.init_node("laser_to_pose_converter")

        # 创建订阅者和发布者
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)

        # 保存当前的位姿信息
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"

        # 设置位姿的初始方向（四元数）
        q = quaternion_from_euler(0, 0, 0)  # RPY: 0, 0, 0
        self.current_pose.pose.orientation.x = q[0]
        self.current_pose.pose.orientation.y = q[1]
        self.current_pose.pose.orientation.z = q[2]
        self.current_pose.pose.orientation.w = q[3]

    def laser_callback(self, msg):
        if len(msg.ranges) > 0:
            # 获取激光测距的值
            height = msg.ranges[0]  # 假设使用第一个测距值

            # 更新位姿消息
            self.current_pose.header.stamp = rospy.Time.now()
            self.current_pose.pose.position.x = self.current_pose.pose.position.x  # 保持当前x值
            self.current_pose.pose.position.y = self.current_pose.pose.position.y  # 保持当前y值
            self.current_pose.pose.position.z = height  # 更新高度值

            # 发布位姿消息
            self.pose_pub.publish(self.current_pose)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        converter = LaserToPoseConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
