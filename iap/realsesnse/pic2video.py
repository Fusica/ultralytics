#将一个文件夹中的图片拼接成视频
import cv2
import os
from tqdm import tqdm

def pic2video(pic_path, video_path, fps):
    # 读取图片文件夹中的图片
    pics = os.listdir(pic_path)
    pics.sort(key=lambda x: int(x[:-4]))
    # 获取图片尺寸
    img = cv2.imread(os.path.join(pic_path, pics[0]))
    img_height, img_width = img.shape[:2]
    # 设置视频编码格式
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # 创建视频对象
    video = cv2.VideoWriter(video_path, fourcc, fps, (img_width, img_height))
    # 逐帧写入视频
    for pic in tqdm(pics):
        img = cv2.imread(os.path.join(pic_path, pic))
        video.write(img)
    video.release()
    print(f"Video saved to {video_path}")

# 使用示例
pic_path = "/Volumes/DATA/track_vis"
video_path = "/Users/max/Downloads/track_vis.mp4"
fps = 30
pic2video(pic_path, video_path, fps)
