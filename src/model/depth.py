import numpy as np
from sensor_msgs.msg import Image, CameraInfo

def calculate_camera_height(image_msg, camera_info_msg):
    """
    根据深度图像和相机参数计算无人机到图像中心点的距离。

    参数:
        image_msg: sensor_msgs/Image 消息，包含深度数据（32FC1编码）。
        camera_info_msg: sensor_msgs/CameraInfo 消息，包含相机内参。
    返回:
        无人机到图像中心点的距离（单位：米）。
    """
    # 将深度图像消息转换为 NumPy 数组
    depth_image = np.frombuffer(image_msg.data, dtype=np.float32).reshape(image_msg.height, image_msg.width)

    # 计算图像中心点的像素坐标
    center_x = image_msg.width // 2
    center_y = image_msg.height // 2

    # 获取图像中心点的深度值
    center_depth = depth_image[center_y, center_x]

    # 检查深度值是否有效（非NaN、非Inf且非零）
    if not np.isfinite(center_depth) or center_depth == 0:
        raise ValueError("图像中心点的深度值无效。")

    # 对于朝向正下方的相机，深度值直接表示无人机到地面的距离
    return center_depth

