# Dataset Creation Tools

## 注意：一定要保留原图数据集！！！

后两步使用是由于使用roboflow会默认压缩图片，所以在roboflow标注好数据后，可以使用本目录的工具来替换原本图片，来构建原图数据集。

## 工具概述

### 1a. step_0_slice_video.py
该脚本用于将视频文件切分为单独的帧图像。

用法：
```bash
python step_0_slice_bag.py [video_path] [output_folder] [-i INTERVAL]
```

参数说明：
- `video_path`: 输入视频文件的路径
- `output_folder`: 输出帧图像的目录
- `-i, --interval`: 帧间隔（默认为1，表示每一帧都保存）

示例：
```bash
python step_0_slice_video.py input.mp4 output_frames -i 5
```

### 1b. step_0_slice_bag.py
该脚本用于从ROS的bag文件中提取图像帧。

用法：
```bash
python step_0_slice_bag.py [bag_file] [output_folder] [--topic TOPIC_NAME]
```

参数说明：
- `bag_file`: 输入的ROS bag文件路径
- `output_folder`: 输出帧图像的目录
- `--topic`: 要提取的图像话题名称

注意事项：
- 需要安装rosbag和cv_bridge依赖
- 确保bag文件包含图像消息
- 图像话题应该是sensor_msgs/Image类型

### 2. step_1_revert_file_name.py
该脚本用于标注后的文件名规范化，将可能被修改的文件名恢复为原始的帧编号格式。

用法：
```bash
python step_1_revert_file_name.py
```

注意事项：
- 使用前需要在脚本中修改 `root_directory` 变量为你的数据集根目录
- 脚本会处理 .jpg 和 .txt 文件
- 文件名格式必须包含 "frame_XXXXXX" 的模式（X为数字）

### 3. step_2_rename_replace.py
该脚本用于替换数据集中的图片文件，通常用于将压缩后的图片替换为原始高质量图片。

用法：
```bash
python step_2_rename_replace.py
```

注意事项：
- 使用前需要在脚本中修改以下变量：
  - `dataset_root`: 数据集的根目录
  - `original_images_dir`: 原始图片所在的目录
- 确保原始图片和待替换图片的文件名完全匹配
- 脚本会保持目录结构不变，只替换图片文件

## 使用流程

1. 首先使用 `step_0_slice_video.py` 或 `step_0_slice_bag.py` 将数据源转换为帧序列
2. 对帧图像进行标注工作
3. 如果标注过程中文件名发生变化，使用 `step_1_revert_file_name.py` 恢复标准文件名
4. 如果需要替换为原始高质量图片，使用 `step_2_replace_img.py` 进行替换

## 注意事项

1. 在运行脚本之前，请确保已安装所需的依赖：
   ```bash
   pip install opencv-python
   # 如果使用bag文件提取，还需要：
   pip install rosbag cv_bridge
   ```

2. 建议在处理大量文件之前，先在小规模数据集上测试脚本功能

3. 请确保有足够的磁盘空间存储处理后的图片文件

4. 建议在执行替换操作前备份重要数据

5. 所有脚本都支持中文路径，但建议使用英文路径以避免潜在的编码问题

