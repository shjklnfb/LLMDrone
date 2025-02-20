#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
from geometry_msgs.msg import TwistStamped

def send_velocity_command(prefix, vel_x, vel_y, vel_z, duration):
    node_name = f'{prefix}_velocity_command_sender'
    # rospy.init_node(node_name, anonymous=True,)
    command_topic = f'/{prefix}/drone/velocity_command'
    command_pub = rospy.Publisher(command_topic, TwistStamped, queue_size=10)
    rospy.sleep(1) 
    
    twist = TwistStamped()
    twist.twist.linear.x = vel_x
    twist.twist.linear.y = vel_y
    twist.twist.linear.z = vel_z
    twist.header.stamp = rospy.Time.now() + rospy.Duration(duration)  # 将飞行时间作为时间戳
    rospy.loginfo(f"Publishing velocity command: ({vel_x}, {vel_y}, {vel_z}) for {duration} seconds")
    command_pub.publish(twist)

if __name__ == '__main__':
    if len(sys.argv) != 6:
        print("Usage: python send_velocity_command.py <prefix> <vel_x> <vel_y> <vel_z> <duration> ")
        sys.exit(1)

    prefix = sys.argv[1]
    vel_x = float(sys.argv[2])
    vel_y = float(sys.argv[3])
    vel_z = float(sys.argv[4])
    duration = float(sys.argv[5])

    send_velocity_command(prefix, vel_x, vel_y, vel_z, duration)
