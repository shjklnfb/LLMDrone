import threading
import time
# import rospy
# from mavros_msgs.msg import State
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image
# from mavros_msgs.srv import CommandBool

# 无人机监控器类
class DroneMonitor(threading.Thread):
    def __init__(self, drone_id, shared_data):
        super().__init__()
        self.drone_id = drone_id
        self.shared_data = shared_data

    def run(self):
        self.monitor_drone()

    def state_cb(self, state):
        self.shared_data[self.drone_id]['state'] = state
        print(f"Drone {self.drone_id} state: {state}")

    def local_pos_cb(self, odom):
        self.shared_data[self.drone_id]['position'] = odom.pose.pose.position
        self.shared_data[self.drone_id]['velocity'] = odom.twist.twist.linear
        print(f"Drone {self.drone_id} position: {odom.pose.pose.position}, velocity: {odom.twist.twist.linear}")

    def image_cb(self, image):
        self.shared_data[self.drone_id]['image'] = image
        print(f"Drone {self.drone_id} received image data")

    # 监控无人机
    def monitor_drone(self):
        print(f"Starting monitoring for drone {self.drone_id}")
        time.sleep(3)
        # rospy.init_node(f'drone_monitor_{self.drone_id}', anonymous=True)
        # prefix = f'{self.drone_id}'
        
        # state_sub = rospy.Subscriber(f'/{prefix}/mavros/state', State, self.state_cb)
        # local_pos_sub = rospy.Subscriber(f'/{prefix}/mavros/local_position/odom', Odometry, self.local_pos_cb)
        # image_sub = rospy.Subscriber(f'/{prefix}/camera/image_raw', Image, self.image_cb)
        # arming_client = rospy.ServiceProxy(f'/{prefix}/mavros/cmd/arming', CommandBool)
        
        # rate = rospy.Rate(1)  # 1 Hz
        # while not rospy.is_shutdown():
        #     # Here you can add any additional monitoring logic if needed
        #     rate.sleep()
