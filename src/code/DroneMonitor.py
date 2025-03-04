import threading
import time
import os
from datetime import datetime
import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from mavros_msgs.srv import CommandBool
from src.llm.ImageRecognition import analyze_image,analyze_depth_image

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
        step = self.shared_data[self.drone_id].get('step', 0)
        # analysis_result = analyze_image(image,step=step)
        self.shared_data[self.drone_id]['image'] = image
        # self.log(f"Drone {self.drone_id} image analysis result: {image}")

    def depth_image_cb(self, image):
        self.log(f"Drone {self.drone_id} received depth image data")
        step = self.shared_data[self.drone_id].get('step', 0)
        # analysis_result = analyze_depth_image(image, step)
        self.shared_data[self.drone_id]['depth_image'] = image
        # self.log(f"Drone {self.drone_id} depth_image analysis result: {image}")

    def depth_camera_info_cb(self, camera_info):
        self.shared_data[self.drone_id]['camera_info'] = camera_info
        self.log(f"Drone {self.drone_id} received depth camera info")
        
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
        arming_client = rospy.ServiceProxy(f'/{prefix}/mavros/cmd/arming', CommandBool)
        
        # 每秒执行一次，将无人机的状态、位置、速度、图像数据等信息存储到共享数据中
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rate.sleep()
