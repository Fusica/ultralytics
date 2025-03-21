import cv2
import h5py
import numpy as np
import os
from tqdm import tqdm  # 用于显示进度条

class VideoSlicer:
    def __init__(self, video_path, depth_path):
        """
        初始化切片器
        :param video_path: RGB视频路径
        :param depth_path: 深度数据h5文件路径
        """
        if not os.path.exists(video_path) or not os.path.exists(depth_path):
            raise FileNotFoundError("视频文件或深度文件不存在！")
            
        self.video = cv2.VideoCapture(video_path)
        self.depth_file = h5py.File(depth_path, 'r')
        self.total_frames = int(self.video.get(cv2.CAP_PROP_FRAME_COUNT))
        
    def slice_all(self):
        """
        将视频和深度数据切片为单独的帧
        :return: rgb_frames, depth_frames (两个列表，分别包含所有的RGB帧和深度帧)
        """
        rgb_frames = []
        depth_frames = []
        
        print("开始读取并切片视频和深度数据...")
        for frame_idx in tqdm(range(self.total_frames)):
            # 读取RGB帧
            ret, frame = self.video.read()
            if not ret:
                break
                
            # 读取对应的深度帧
            depth_name = f'{str(frame_idx).zfill(5)}_depth.png'
            if depth_name in self.depth_file:
                depth_data = self.depth_file[depth_name][:]
                depth_frame = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)
                
                # 检查深度图是否为16位
                if depth_frame.dtype != np.uint16:
                    print(f"警告：第{frame_idx}帧深度图不是16位格式！当前格式：{depth_frame.dtype}")
                    # 如果需要转换为16位
                    depth_frame = depth_frame.astype(np.uint16)
                
                rgb_frames.append(frame)
                depth_frames.append(depth_frame)
            
        print(f"切片完成！共处理 {len(rgb_frames)} 帧")
        return rgb_frames, depth_frames
    
    def close(self):
        """释放资源"""
        self.video.release()
        self.depth_file.close()

def process_video(video_path, depth_path):
    """
    处理视频和深度数据的主函数
    :param video_path: RGB视频路径
    :param depth_path: 深度数据h5文件路径
    """
    try:
        # 创建保存目录
        current_dir = os.path.dirname(os.path.abspath(__file__))  # 获取当前脚本所在目录
        rgb_dir = os.path.join(current_dir, 'rgb_frames')
        depth_dir = os.path.join(current_dir, 'depth_frames')
        
        os.makedirs(rgb_dir, exist_ok=True)
        os.makedirs(depth_dir, exist_ok=True)
        
        slicer = VideoSlicer(video_path, depth_path)
        
        print("开始读取并保存帧...")
        frame_count = 0
        
        # 获取视频总帧数
        total_frames = int(slicer.video.get(cv2.CAP_PROP_FRAME_COUNT))
        
        for frame_idx in tqdm(range(total_frames)):
            # 读取RGB帧
            ret, frame = slicer.video.read()
            if not ret:
                break
                
            # 读取对应的深度帧
            depth_name = f'{str(frame_idx).zfill(5)}_depth.png'
            if depth_name in slicer.depth_file:
                depth_data = slicer.depth_file[depth_name][:]
                depth_frame = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)
                
                # 检查深度图是否为16位
                if depth_frame.dtype != np.uint16:
                    print(f"警告：第{frame_idx}帧深度图不是16位格式！当前格式：{depth_frame.dtype}")
                    depth_frame = depth_frame.astype(np.uint16)
                
                # 保存RGB帧
                rgb_path = os.path.join(rgb_dir, f'frame_{frame_idx:05d}.png')
                cv2.imwrite(rgb_path, frame)
                
                # 保存深度帧
                depth_path = os.path.join(depth_dir, f'depth_{frame_idx:05d}.png')
                cv2.imwrite(depth_path, depth_frame)
                
                frame_count += 1
        
        slicer.close()
        
        # 打印处理信息
        print(f"\n处理完成！")
        print(f"总共处理了 {frame_count} 帧")
        print(f"RGB图像保存在: {rgb_dir}")
        print(f"深度图保存在: {depth_dir}")
        
        # 打印第一帧的信息（如果存在）
        first_rgb = os.path.join(rgb_dir, 'frame_00000.png')
        first_depth = os.path.join(depth_dir, 'depth_00000.png')
        if os.path.exists(first_rgb) and os.path.exists(first_depth):
            rgb_sample = cv2.imread(first_rgb)
            depth_sample = cv2.imread(first_depth, cv2.IMREAD_UNCHANGED)
            print(f"\n数据信息:")
            print(f"RGB图像尺寸: {rgb_sample.shape}")
            print(f"深度图尺寸: {depth_sample.shape}")
            print(f"深度图数据类型: {depth_sample.dtype}")
            print(f"深度值范围: {np.min(depth_sample)} - {np.max(depth_sample)}")
        
    except Exception as e:
        print(f"处理过程中出现错误: {str(e)}")

# 使用示例
if __name__ == "__main__":
    video_path = "D://Realsense//Video//uav//targetvideo_rgb.mp4"
    depth_path = "D://Realsense//Video//uav//targetvideo_depth.h5"
    
    process_video(video_path, depth_path)
