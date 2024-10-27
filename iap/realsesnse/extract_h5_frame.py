import os
import h5py  # 用于16位深度图视频的读取与分解
import cv2
import numpy as np


def h52img(input_depth, output_depth):
    # input_depth : h5文件路径
    # output_depth: 图片文件路径
    h5 = h5py.File(input_depth, 'r')  # 读取深度图视频
    os.path.join(output_depth)
    cnt = 0
    for key in h5.keys():
        cnt = cnt + 1  # 每一帧都进行深度图保存
        img = cv2.imdecode(np.array(h5[key]), -1)  # 16位深度图解码
        img_name = str(cnt) + '.png'  # 自定义的深度图文件名称
        cv2.imwrite(os.path.join(output_depth, img_name), img)  # 以png格式保存深度图
    h5.close()


if __name__ == '__main__':
    # 目标深度图视频读取路径，需要根据本地情况进行修改
    input_depth = r'/Volumes/DATA/Realsense/Video/1727529580/targetvideo_depth.h5'
    # 深度图图像帧保存路径
    output_depth = r'/Volumes/DATA/Realsense/Video/1727529580/depth'
    # 创建保存路径对应的文件夹
    if os.path.exists(output_depth):
        print('depth文件夹存在')
    else:
        print('depth文件夹缺失,自动创建 ')
        os.mkdir(output_depth)
    # 调用视频分解函数，将目标视频分解为图像帧
    h52img(input_depth, output_depth)