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

import numpy as np
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
import tf

def calculate_object_coordinates(depth_image_msg, camera_info_msg, x_center, y_center, drone_position, imu_data_msg):
    """
    根据深度图像、相机参数和IMU数据计算标注框中物体的实际坐标。

    参数:
        depth_image_msg: sensor_msgs/Image 消息，包含深度数据（32FC1编码）。
        camera_info_msg: sensor_msgs/CameraInfo 消息，包含相机内参。
        x_center: 标注框中心点的x坐标（像素）。
        y_center: 标注框中心点的y坐标（像素）。
        drone_position: 无人机的当前位置（geometry_msgs/Point）。
        imu_data_msg: sensor_msgs/Imu 消息，包含无人机的IMU数据，包括姿态四元数。
    返回:
        标注框中物体的实际坐标（单位：米）。
    """
    # 将深度图像消息转换为 NumPy 数组
    depth_image = np.frombuffer(depth_image_msg.data, dtype=np.float32).reshape(depth_image_msg.height, depth_image_msg.width)

    # 检查标注框中心点的坐标是否在图像范围内
    if int(y_center) < 0 or int(y_center) >= depth_image.shape[0] or int(x_center) < 0 or int(x_center) >= depth_image.shape[1]:
        raise ValueError("标注框中心点的坐标超出图像范围。")

    # 获取标注框中心点的深度值
    object_depth = depth_image[int(y_center), int(x_center)]

    # 检查深度值是否有效（非NaN、非Inf且非零）
    if not np.isfinite(object_depth) or object_depth == 0:
        raise ValueError("标注框中心点的深度值无效。")

    # 计算相机内参
    fx = camera_info_msg.K[0]
    fy = camera_info_msg.K[4]
    cx = camera_info_msg.K[2]
    cy = camera_info_msg.K[5]

    # 计算物体在相机坐标系中的坐标
    x = (x_center - cx) * object_depth / fx
    y = (y_center - cy) * object_depth / fy
    z = object_depth

    # 将相机坐标系中的坐标转换为世界坐标系中的坐标
    target_x = drone_position.x + x
    target_y = drone_position.y + y
    target_z = drone_position.z - z  # assuming downward-facing camera

    # 获取IMU数据中的姿态四元数
    quaternion = (imu_data_msg.orientation.x, imu_data_msg.orientation.y, imu_data_msg.orientation.z, imu_data_msg.orientation.w)

    # 将四元数转换为旋转矩阵
    R = tf.transformations.quaternion_matrix(quaternion)[:3, :3]

    # 将相机坐标系中的坐标转换为世界坐标系中的坐标
    # 注意：这里假设相机坐标系的z轴指向世界坐标系的负z轴
    world_coordinates = R.dot(np.array([target_x, target_y, -target_z]))

    return world_coordinates[0], world_coordinates[1], world_coordinates[2]