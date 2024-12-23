import os
import random
import shutil
from pathlib import Path

def sample_pairs(img_root, label_root, output_dir, samples_per_folder=500, min_interval=30):
    """
    从每个子文件夹随机抽取样本对，保证时间间隔
    
    Args:
        img_root: 图片根目录
        label_root: 标签根目录
        output_dir: 输出目录
        samples_per_folder: 每个子文件夹抽取的样本数
        min_interval: 最小时间间隔(按文件名序号)
    """
    
    # 创建输出目录
    output_img_dir = Path(output_dir) / "images"
    output_label_dir = Path(output_dir) / "labels"
    output_img_dir.mkdir(parents=True, exist_ok=True)
    output_label_dir.mkdir(parents=True, exist_ok=True)
    
    # 获取所有子文件夹
    img_folders = [f for f in os.listdir(img_root) if os.path.isdir(os.path.join(img_root, f))]
    
    for folder in img_folders:
        img_folder = Path(img_root) / folder
        label_folder = Path(label_root) / folder / "labels"
        
        if not label_folder.exists():
            label_folder = Path(label_root) / folder
            if not label_folder.exists():
                print(f"警告: {label_folder} 不存在，跳过")
                continue
        
        # 获取所有图片文件
        img_files = sorted([f for f in os.listdir(img_folder) if f.endswith(('.jpg', '.png'))])
        
        if len(img_files) < samples_per_folder:
            print(f"警告: {folder} 中的样本数量不足 {samples_per_folder}, 实际数量: {len(img_files)}")
            continue
        
        # 调整采样策略
        n_samples = min(samples_per_folder, len(img_files))
        total_frames = len(img_files)
        
        # 计算实际可用的间隔
        effective_interval = min(min_interval, total_frames // (n_samples * 2))
        
        valid_indices = list(range(len(img_files)))
        selected_indices = []
        
        # 随机选择索引，使用动态间隔
        while len(selected_indices) < n_samples and valid_indices:
            idx = random.choice(valid_indices)
            
            # 使用较小的间隔
            invalid_range = range(max(0, idx - effective_interval), 
                                min(len(img_files), idx + effective_interval + 1))
            valid_indices = [i for i in valid_indices if i not in invalid_range]
            
            selected_indices.append(idx)
            
            # 如果剩余的有效索引太少，减小间隔
            if len(valid_indices) < (n_samples - len(selected_indices)) * 2:
                effective_interval = max(1, effective_interval // 2)
                valid_indices = [i for i in range(len(img_files)) 
                               if i not in set(selected_indices)]
        
        # 复制选中的文件
        for idx in selected_indices:
            img_file = img_files[idx]
            label_file = img_file.rsplit('.', 1)[0] + '.txt'  # 假设标签文件与图片同名，扩展名为.txt
            
            # 构建新的文件名，添加文件夹前缀以避免重名
            new_img_name = f"{folder}_{img_file}"
            new_label_name = f"{folder}_{label_file}"
            
            # 复制文件
            shutil.copy2(img_folder / img_file, output_img_dir / new_img_name)
            if (label_folder / label_file).exists():
                shutil.copy2(label_folder / label_file, output_label_dir / new_label_name)
            else:
                print(f"警告: 标签文件不存在 {label_folder / label_file}")
        
        print(f"已处理 {folder}: 选择了 {len(selected_indices)} 个样本")

if __name__ == "__main__":
    # 设置路径 - 根据实际路径修改
    img_root = "/Volumes/Data_WD/弹群检测/十二月新数据/images"
    label_root = "/Volumes/Data_WD/弹群检测/十二月新数据/labels"
    output_dir = "/Volumes/Data_WD/弹群检测/十二月新数据/sampled_dataset"
    
    # 减小初始间隔
    sample_pairs(img_root, label_root, output_dir, samples_per_folder=500, min_interval=15) 