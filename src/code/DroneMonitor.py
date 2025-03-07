import threading
import time
import os
from datetime import datetime
import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Imu  # Add this import
from src.llm.ImageRecognition import analyze_image,analyze_depth_image
from cv_bridge import CvBridge
import cv2
from src.model.depth import calculate_object_coordinates

# 无人机监控器类
class DroneMonitor(threading.Thread):
    def __init__(self, drone_id, shared_data):
        super().__init__()
        self.drone_id = drone_id
        self.shared_data = shared_data
        self.log_file = self.create_log_file()

    def create_log_file(self):
        log_dir = os.path.join(os.getcwd(), 'log', 'monitor')
        os.makedirs(log_dir, exist_ok=True)
        log_filename = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_drone_{self.drone_id}.log'
        return open(os.path.join(log_dir, log_filename), 'a')

    def log(self, message):
        self.log_file.write(message + '\n')
        self.log_file.flush()

    def run(self):
        self.monitor_drone()

    def state_cb(self, state):
        self.shared_data[self.drone_id]['state'] = state
        self.log(f"Drone {self.drone_id} state: {state}")

    def local_pos_cb(self, odom):
        self.shared_data[self.drone_id]['position'] = odom.pose.pose.position
        self.shared_data[self.drone_id]['velocity'] = odom.twist.twist.linear
        self.log(f"Drone {self.drone_id} position: {odom.pose.pose.position}, velocity: {odom.twist.twist.linear}")

    def image_cb(self, image):
        self.log(f"Drone {self.drone_id} received image data")
        self.shared_data[self.drone_id]['image'] = image
        # 图像处理
        from src.model.yolo.yolov3.search import detect_target
        judge,position = detect_target(image, 'person')
        
        if judge:
            
            self.log(f"Detected car in image from drone {self.drone_id}")
            # 保存普通图像
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
            # image_filename = os.path.join(os.getcwd(), 'log', 'monitor', f'drone_{self.drone_id}_image_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jpg')
            # cv2.imwrite(image_filename, cv_image)
            # self.log(f"Saved image from drone {self.drone_id} to {image_filename}")

            # 根据标注框和深度图像，计算目标位置
            depth_image = self.shared_data[self.drone_id].get('depth_image')
            camera_info = self.shared_data[self.drone_id].get('camera_info')
            if depth_image and camera_info:

                # 计算目标坐标
                try:
                    drone_position = self.shared_data[self.drone_id]['position']
                    image_width = image.width
                    image_height = image.height
                    x_center_norm, y_center_norm, width_norm, height_norm = position
                    x_center = x_center_norm
                    y_center = y_center_norm
                    target_coordinates = calculate_object_coordinates(depth_image, camera_info, x_center, y_center, drone_position, self.shared_data[self.drone_id]['imu_data'])
                    self.shared_data[self.drone_id]['target_position'] = target_coordinates
                    # print(f"目标坐标：{target_coordinates}")
                    self.log(f"Calculated target position: {target_coordinates}")
                    self.shared_data['other'] = "经过目标检测,发现人"+"目标坐标："+str(target_coordinates)+"寻找人的步骤已经得到满足，可以返回真"
                except ValueError as e:
                    self.log(f"Error calculating target position: {e}")

    def depth_image_cb(self, image):
        self.log(f"Drone {self.drone_id} received depth image data")
        self.shared_data[self.drone_id]['depth_image'] = image

    def depth_camera_info_cb(self, camera_info):
        self.shared_data[self.drone_id]['camera_info'] = camera_info
        self.log(f"Drone {self.drone_id} received depth camera info")
        
    def imu_data_cb(self, imu_data):
        self.shared_data[self.drone_id]['imu_data'] = imu_data  # Store the entire IMU data
        self.log(f"Drone {self.drone_id} received IMU data: {imu_data}")

    # 监控无人机
    def monitor_drone(self):
        self.log(f"Starting monitoring for drone {self.drone_id}")
        # rospy.init_node(f'drone_monitor_{self.drone_id}', anonymous=True)
        prefix = f'{self.drone_id}'
        
        state_sub = rospy.Subscriber(f'/{prefix}/mavros/state', State, self.state_cb)
        local_pos_sub = rospy.Subscriber(f'/{prefix}/mavros/local_position/odom', Odometry, self.local_pos_cb)
        image_sub = rospy.Subscriber(f'/{prefix}/realsense/depth_camera/color/image_raw', Image, self.image_cb)
        depth_image_sub = rospy.Subscriber(f'/{prefix}/realsense/depth_camera/depth/image_raw', Image, self.depth_image_cb)
        depth_camera_info_sub = rospy.Subscriber(f'/{prefix}/realsense/depth_camera/depth/camera_info', CameraInfo, self.depth_camera_info_cb)
        imu_data_sub = rospy.Subscriber(f'/{prefix}/mavros/imu/data', Imu, self.imu_data_cb)
        arming_client = rospy.ServiceProxy(f'/{prefix}/mavros/cmd/arming', CommandBool)
        
        # 每秒执行一次，将无人机的状态、位置、速度、图像数据等信息存储到共享数据中
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rate.sleep()
