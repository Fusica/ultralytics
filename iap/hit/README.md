# 打击检测节点启动


## 启动相机
cd iap_sh_fy
./d435i.sh

## 运行检测节点，发送/center_zed消息
conda activate yolo
cd /home/robot/ultralytics/detect\&hit/
python bbox_node.py

## 记录bag
rosbag record /camera/color/image_raw /camera/depth/image_rect_raw /center_zed

## PARAMS:
CONFIDENCE_THRESHOLD 设置置信度阈值，如果太多非防弹甲被检测，尝试增大该数值

### PS：需要去掉cv2可视化，以避免ssh运行时出现没有可视化的问题
