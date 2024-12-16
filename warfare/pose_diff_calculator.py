#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
import numpy as np

class PoseDiffCalculator:
    def __init__(self):
        rospy.init_node('pose_diff_calculator')
        
        self.target_pose = None
        self.uav_pose = None
        
        # 订阅两个pose话题
        rospy.Subscriber('/vrpn_client_node/target/pose', PoseStamped, self.target_callback)
        rospy.Subscriber('/vrpn_client_node/uav/pose', PoseStamped, self.uav_callback)
        
        # 设置循环频率
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 添加发布器
        self.diff_pub = rospy.Publisher('/pose_diff', Point, queue_size=10)
        
    def target_callback(self, msg):
        self.target_pose = msg
        
    def uav_callback(self, msg):
        self.uav_pose = msg
        
    def calculate_diff(self):
        if self.target_pose is None or self.uav_pose is None:
            return None
            
        diff = np.array([
            self.target_pose.pose.position.x - self.uav_pose.pose.position.x,
            self.target_pose.pose.position.y - self.uav_pose.pose.position.y,
            self.target_pose.pose.position.z - self.uav_pose.pose.position.z - 0.14
        ])
        
        return diff
        
    def run(self):
        while not rospy.is_shutdown():
            diff = self.calculate_diff()
            if diff is not None:
                # 发布 diff
                diff_point = Point()
                diff_point.x = diff[0]
                diff_point.y = diff[1]
                diff_point.z = diff[2]
                self.diff_pub.publish(diff_point)
                
                rospy.loginfo(f"Position difference (target - uav): x={diff[0]:.2f}, y={diff[1]:.2f}, z={diff[2]:.2f}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        calculator = PoseDiffCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass 