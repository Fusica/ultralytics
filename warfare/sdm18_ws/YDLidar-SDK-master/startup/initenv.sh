#!/bin/bash

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "请使用 sudo 运行此脚本"
    exit 1
fi

# 创建所有可能的 YDLidar 设备规则
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-V2.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-2303.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-7522.rules

# 添加用户到 dialout 组
current_user=$(who am i | awk '{print $1}')
usermod -a -G dialout $current_user

# 重新加载 udev 规则
udevadm control --reload-rules
udevadm trigger

echo "YDLidar udev 规则已设置完成！"
echo "用户已添加到 dialout 组"
echo "请重新插入设备或重新启动系统使更改生效"