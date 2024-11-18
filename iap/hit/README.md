# 敌方检测与打击操作指南

## **NX 主机操作**

### 1. 启动相机
```bash
cd iap_sh
./d435i.sh
```

### 2. 启动打击节点
```bash
cd ultralytics/detect\&hit/
python bbox_node.py
```

### 3. 启动 Swarm 系统
```bash
cd swarm_ros_bridge_ws
source devel/setup.bash
roslaunch swarm_ros_bridge test.launch
```

### 4. 记录数据 Bag
```bash
rosbag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /center_zed
```

### 5. 检查 Center Zed 值
- 确保 z 值为负数：
```bash
rostopic echo /center_zed
```


## **中心 Nano 主机操作**

### 1. 启动 Swarm 系统
```bash
cd swarm_ros_bridge_ws/
source devel/setup.bash
roslaunch swarm_ros_bridge test.launch
```

---

## **登录信息**
用户名：nvidia
密码：nvidia

