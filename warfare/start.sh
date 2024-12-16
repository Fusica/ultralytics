#!/bin/bash

# 如果已存在同名会话，先清理
tmux kill-session -t siyi 2>/dev/null
tmux kill-session -t cam 2>/dev/null
tmux kill-session -t mavros 2>/dev/null
tmux kill-session -t lidar 2>/dev/null
tmux kill-session -t detect 2>/dev/null

# 调整网口速度
echo "调整网口速度..."
sudo ethtool -s eth0 speed 10 duplex full autoneg off

# 启动云台控制
echo "启动云台控制..."
tmux new-session -d -s siyi "source ~/warfare_ws/devel/setup.bash && roslaunch siyi_controller siyi_rpy.launch || /bin/bash"
sleep 1

# 启动图像发布
echo "启动图像发布..."
tmux new-session -d -s cam "cd ~/ultralytics/warfare && (python capture_webcam.py || echo '相机启动失败') && /bin/bash"
sleep 1

# 启动MAVROS
echo "启动无人机MAVROS..."
tmux new-session -d -s mavros "source ~/warfare_ws/devel/setup.bash && (roslaunch mavros px4.launch fcu_url:=serial:///dev/ttyACM0 || echo 'MAVROS启动失败') && /bin/bash"
sleep 1

# 启动激光测距
echo "启动激光测距..."
tmux new-session -d -s lidar "source ~/ydlidar_ros_ws/devel/setup.bash && roslaunch ydlidar_ros_driver SDM18.launch || /bin/bash"
sleep 1

# 启动目标检测
echo "启动目标检测..."
tmux new-session -d -s detect "cd ~/ultralytics/warfare && python object_detect_pb.py || /bin/bash"

echo "所有会话已启动!"
echo "使用以下命令管理会话:"
echo "- 查看所有会话: tmux ls"
echo "- 接入会话: tmux attach -t <session-name>"
echo "- 退出会话: Ctrl+B 再按 D"
echo "- 结束会话: tmux kill-session -t <session-name>"

# 显示当前会话状态
sleep 3
echo -e "\n当前会话状态："
tmux ls
