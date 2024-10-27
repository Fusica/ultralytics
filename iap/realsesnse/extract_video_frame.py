import cv2  # 引入图像处理与系统操作的库
import os


def process_rgb_video(i_video, o_video):  # 定义抽帧函数
    cap = cv2.VideoCapture(i_video)  # 获取视频
    expand_name = '.png'  # 分解后图片名称后缀
    if not cap.isOpened():  # 视频路径检测
        print("Please check the path.")
    cnt = 0
    count = 0
    num = 1
    while 1:
        ret, frame = cap.read()  # 获取图像帧
        cnt += 1  # 保存每一帧，修改参数连续5帧后抽取一帧并保存该帧图片，具体由使用者设置
        if cnt % num == 0:
            count += 1
            cv2.imwrite(os.path.join(o_video, str(count) + expand_name), frame)
        if not ret:
            cap.close()  # 遍历完视频帧后退出
            break


if __name__ == '__main__':
    # 读取视频路径，需要根据本地情况进行修改
    input_rgb = r'/Volumes/DATA/Realsense/Video/1727529580/targetvideo_rgb.mp4'
    # 输出图像帧路径，可根据需求进行修改
    output_rgb = r'/Volumes/DATA/Realsense/Video/1727529580/rgb'
    # 创建输出路径文件夹
    if os.path.exists(output_rgb):
        print('rgb文件夹存在')
    else:
        print('rgb文件夹缺失,自动创建 ')
        os.mkdir(output_rgb)
    # 调用图像抽帧函数，分解RGB视频
    process_rgb_video(input_rgb, output_rgb)