# 敌方检测与打击

# **打击NX**

用户名：nvidia

密码：nvidia

## **启动相机**

cd iap_sh
./d435i.sh

---

## **启动打击节点**

cd ultralytics/detect\&hit/
python bbox_node.py

---

## **启动swarm**

cd swarm_ros_bridge_ws
source devel/setup.bash
roslaunch swarm_ros_bridge test.launch

---

## **记录bag**

rosbag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /center_zed

---

## **检查center_zed，z为负值**

rostopic echo /center_zed

---

---

# **中心nano**

## **启动swarm**

cd swarm_ros_bridge_ws/
source devel/setup.bash
roslaunch swarm_ros_bridge test.launch