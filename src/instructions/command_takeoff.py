#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import queue
import sys
from geometry_msgs.msg import TwistStamped, PoseStamped,Twist
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import math
import random

#TODO 日志管理
#TODO 增加新指令：？
#TODO 调整优先级，范围0-10，默认10，突发0，恢复1，2-9可选

class CommandTakeoff:
    def __init__(self, namespace):
        self.current_state = State()  # 当前状态
        self.local_pos = Odometry()  # 当前位置
        self.command_queue = queue.Queue()  # 普通队列
        self.pose = PoseStamped()  # 当前目标位置
        self.current_target_pose = PoseStamped()  # 用于存储目标位置
        self.is_executing = False  # 标志是否正在执行指令
        self.current_command = None  # 当前执行的指令
        self.past_command = None  # 当前执行的指令
        self.prefix = namespace  # 命名空间
        self.clear_queue_flag = False

    # 状态回调函数
    def state_cb(self, msg):
        self.current_state = msg

    # 位置回调函数
    def local_pos_cb(self, msg):
        self.local_pos = msg

    # 清空队列标志回调函数 
    def clear_queue_cb(self, msg): 
        self.clear_queue_flag = msg.data 
        rospy.loginfo(f"Received clear queue flag: {self.clear_queue_flag}")

    # 位置指令回调函数
    def command_cb(self, msg):
        command = {'type': 'position', 'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z}  
        rospy.loginfo(f"Received position command: {command}")  # 打印接收到的指令
        self.command_queue.put(command)

    # 速度指令回调函数
    def velocity_command_cb(self, msg):
        command = {'type': 'velocity', 'vel_x': msg.twist.linear.x, 'vel_y': msg.twist.linear.y, 'vel_z': msg.twist.linear.z, 'duration': msg.header.stamp.to_sec() - rospy.Time.now().to_sec()}
        rospy.loginfo(f"Received velocity command: {command}")  # 打印接收到的指令
        self.command_queue.put(command)

    # 降落指令回调函数
    def land_command_cb(self, msg):
        command = {'type': 'land'}
        rospy.loginfo(f"Received land command: {command}")  # 打印接收到的指令
        self.command_queue.put(command)

    # 悬停指令回调函数
    def hover_command_cb(self, msg):
        command = {'type': 'hover', 'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z, 'time': msg.pose.orientation.z}
        rospy.loginfo(f"Received hover command: {command}")  # 打印接收到的指令
        self.command_queue.put(command)

    # 旋转指令回调函数
    def rotate_command_cb(self, msg):
        command = {'type': 'rotate', 'yaw_rate': msg.twist.angular.z, 'duration': msg.twist.linear.x}
        rospy.loginfo(f"Received rotate command: {command}")  # 打印接收到的指令
        self.command_queue.put(command)

    # 随机飞行指令回调函数
    def random_flight_command_cb(self, msg):
        command = {'type': 'random_flight', 'radius': msg.pose.position.x, 'duration': msg.pose.position.y}
        rospy.loginfo(f"Received random flight command: {command}")  # 打印接收到的指令
        self.command_queue.put(command)

    # 执行指令
    def execute_command(self, command):
        rospy.loginfo(f"Executing command: {command}")  # 输出正在执行的指令
        pose = PoseStamped()
        self.current_command = command
        
        # 如果是速度指令
        if command['type'] == 'velocity':
            rospy.loginfo(f"Publishing velocity: {command['vel_x']}, {command['vel_y']}, {command['vel_z']} for {command['duration']} seconds")
            velocity_pub = rospy.Publisher(f'/{self.prefix}/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
            velocity_msg = TwistStamped()
            velocity_msg.twist.linear.x = command['vel_x']
            velocity_msg.twist.linear.y = command['vel_y']
            velocity_msg.twist.linear.z = command['vel_z']
            rate = rospy.Rate(20.0)
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rospy.Duration(command['duration']):
                if not self.is_executing:
                    rospy.loginfo("Command interrupted")
                    break
                velocity_pub.publish(velocity_msg)
                rate.sleep()
            rospy.loginfo("Velocity command executed")

            # 获取当前位置并设置为目标位置
            pose.pose.position.x = self.local_pos.pose.pose.position.x
            pose.pose.position.y = self.local_pos.pose.pose.position.y
            pose.pose.position.z = self.local_pos.pose.pose.position.z
            self.current_target_pose = pose  # 保存目标位置

        # 如果是位置指令
        elif command['type'] == 'position':
            rospy.loginfo(f"Publishing position: {command['x']}, {command['y']}, {command['z']}")
            position_pub = rospy.Publisher(f'/{self.prefix}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            pose.pose.position.x = command['x']
            pose.pose.position.y = command['y']
            pose.pose.position.z = command['z']
            self.current_target_pose = pose  # 保存目标位置
            rate = rospy.Rate(20.0)
            for _ in range(100):  # 持续发布位置指令
                if not self.is_executing:
                    rospy.loginfo("Command interrupted")
                    break
                position_pub.publish(pose)
                rate.sleep()
            rospy.loginfo("Position command executed")

        # 如果是降落指令
        elif command['type'] == 'land':
            rospy.loginfo("Executing landing command")
            land_client = rospy.ServiceProxy(f'/{self.prefix}/mavros/cmd/land', CommandTOL)
            if land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0):
                rospy.loginfo("Landing command executed")
            else:
                rospy.loginfo("Landing command failed")

        # 如果是旋转指令
        elif command['type'] == 'rotate':
            rospy.loginfo(f"Executing rotate command: rate={command['yaw_rate']} radians/sec for {command['duration']} seconds")
            velocity_pub = rospy.Publisher(f'/{self.prefix}/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
            twist_msg = TwistStamped()
            twist_msg.twist.angular.z = command['yaw_rate']
            rate = rospy.Rate(20)  # 10 Hz
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rospy.Duration(command['duration']):
                if not self.is_executing:
                    rospy.loginfo("Command interrupted")
                    break
                print(twist_msg)
                velocity_pub.publish(twist_msg)
                rate.sleep()
            rospy.loginfo("Rotate command executed")

        # 如果是悬停指令
        elif command['type'] == 'hover':
            rospy.loginfo(f"Executing hover command at position: ({command['x']}, {command['y']}, {command['z']}) for {command['time']} seconds") 
            position_pub = rospy.Publisher(f'/{self.prefix}/mavros/setpoint_position/local', PoseStamped, queue_size=10) 
            pose.pose.position.x = command['x'] 
            pose.pose.position.y = command['y'] 
            pose.pose.position.z = command['z'] 
            self.current_target_pose = pose 
            rate = rospy.Rate(20.0) 
            start_time = rospy.Time.now() 
            while rospy.Time.now() - start_time < rospy.Duration(command['time']): 
                if not self.is_executing: 
                    rospy.loginfo("Command interrupted") 
                    break 
                position_pub.publish(pose) 
                rate.sleep() 
            rospy.loginfo("Hover command executed")

        # 如果是随机飞行指令
        elif command['type'] == 'random_flight':
            rospy.loginfo(f"Executing random flight command in radius: {command['radius']} for {command['duration']} seconds")
            position_pub = rospy.Publisher(f'/{self.prefix}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            rate = rospy.Rate(2)  # 降低更新频率到2 Hz
            max_step_distance = command['radius'] / 5  # 限制每次位置变化的最大距离为半径的1/5
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rospy.Duration(command['duration']):
                if not self.is_executing:
                    rospy.loginfo("Command interrupted")
                    break
                # 生成随机目标位置
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(0, max_step_distance)
                target_x = self.current_target_pose.pose.position.x + distance * math.cos(angle)
                target_y = self.current_target_pose.pose.position.y + distance * math.sin(angle)
                target_z = self.current_target_pose.pose.position.z  # 保持当前高度
                pose.pose.position.x = target_x
                pose.pose.position.y = target_y
                pose.pose.position.z = target_z
                position_pub.publish(pose)
                self.current_target_pose = pose  # 更新当前目标位置
                rate.sleep()
            rospy.loginfo("Random flight command executed")

        # 指令执行完成后，保持目标位置
        self.is_executing = False  # 执行完成，标志更新
        self.current_command = None  # 清除当前指令
        self.hold_position(self.current_target_pose)

    # 保持目标位置
    def hold_position(self, target_pose):
        position_pub = rospy.Publisher(f'/{self.prefix}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rate = rospy.Rate(20.0)
        rospy.loginfo(f"Holding target position: ({target_pose.pose.position.x}, {target_pose.pose.position.y}, {target_pose.pose.position.z})")
        
        for _ in range(5):  # 持续发布目标位置指令，之后返回
            position_pub.publish(target_pose)
            rate.sleep()

    # 每定时检查一次队列并执行指令
    def timer_callback(self, event):
        if not self.command_queue.empty() and not self.is_executing:
            rospy.loginfo(f"Queue size before execution: {self.command_queue.qsize()}")
            command = self.command_queue.get()  # 获取新的指令
            rospy.loginfo(f"Executing command: {command}")
            self.is_executing = True  # 标志指令开始执行
            self.execute_command(command)  # 执行指令
            rospy.loginfo(f"Queue size after execution: {self.command_queue.qsize()}")
        else:
            if self.current_state.armed == 'LAND':
                rospy.loginfo("Landing.")
            else:
                rospy.loginfo("No new command or already executing. Holding position.")
                self.hold_position(self.current_target_pose)  # 确保持续发布目标位置

    # 主函数
    def control_script(self):
        # rospy.init_node('control_script', anonymous=True, disable_signals=True)
        
        # 创建订阅者
        state_sub = rospy.Subscriber(f'/{self.prefix}/mavros/state', State, self.state_cb)
        local_pos_sub = rospy.Subscriber(f'/{self.prefix}/mavros/local_position/odom', Odometry, self.local_pos_cb)
        command_sub = rospy.Subscriber(f'/{self.prefix}/drone/command', PoseStamped, self.command_cb)
        velocity_command_sub = rospy.Subscriber(f'/{self.prefix}/drone/velocity_command', TwistStamped, self.velocity_command_cb)
        land_command_sub = rospy.Subscriber(f'/{self.prefix}/drone/land_command', PoseStamped, self.land_command_cb)
        rotate_command_sub = rospy.Subscriber(f'/{self.prefix}/drone/rotate_command', TwistStamped, self.rotate_command_cb)  # 旋转指令订阅者
        hover_command_sub = rospy.Subscriber(f'/{self.prefix}/drone/hover_command', PoseStamped, self.hover_command_cb)
        random_flight_command_sub = rospy.Subscriber(f'/{self.prefix}/drone/random_flight_command', PoseStamped, self.random_flight_command_cb)  # 新的订阅者
        # 服务客户端
        arming_client = rospy.ServiceProxy(f'/{self.prefix}/mavros/cmd/arming', CommandBool)
        set_mode_client = rospy.ServiceProxy(f'/{self.prefix}/mavros/set_mode', SetMode)
        # 等待飞控连接
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            rate.sleep()

            # 初始位置设定为 (0, 0, 5)
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 5

        for _ in range(100):
            if rospy.is_shutdown():
                break
            rospy.Publisher(f'/{self.prefix}/mavros/setpoint_position/local', PoseStamped, queue_size=10).publish(self.pose)
            rate.sleep()
            
        # 进入 Offboard 模式并解锁
        if set_mode_client(0, "OFFBOARD").mode_sent:
            rospy.loginfo("Offboard mode enabled")
        if arming_client(True).success:
            rospy.loginfo("Vehicle armed")

        # 使用定时器每100ms检查队列并执行指令
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 每0.1秒调用一次 timer_callback

        # 保持主循环活跃
        rospy.spin()

def command_takeoff(namespace):
    takeoff_instance = CommandTakeoff(namespace)
    takeoff_instance.control_script()

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            rospy.logerr("Please provide a prefix as an argument")
            sys.exit(1)
        prefix = sys.argv[1]
        # 启动控制脚本
        command_takeoff(prefix)
    except rospy.ROSInterruptException:
        pass