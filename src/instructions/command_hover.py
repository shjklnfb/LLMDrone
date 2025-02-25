#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
import logging
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# 设置日志记录
log_filename = datetime.now().strftime('../log/flight/command_hover_%Y%m%d_%H%M%S.log')
logging.basicConfig(filename=log_filename, level=logging.INFO)

def send_emergency_hover_command(prefix, duration):
    node_name = f'{prefix}_emergency_hover_command_sender'
    # rospy.init_node(node_name, anonymous=True)
    hover_pub = rospy.Publisher(f'/{prefix}/drone/emergency_hover_command', PoseStamped, queue_size=10)
    rospy.sleep(1)

    def current_position_cb(msg):
        hover_msg = PoseStamped()
        hover_msg.pose.position.x = msg.pose.pose.position.x
        hover_msg.pose.position.y = msg.pose.pose.position.y
        hover_msg.pose.position.z = msg.pose.pose.position.z
        hover_msg.header.stamp = rospy.Time.now()
        hover_msg.pose.orientation.x = duration # 使用 orientation.x 来传递悬停时间

        hover_pub.publish(hover_msg)
        rospy.loginfo(f"Published emergency hover command at position: ({hover_msg.pose.position.x}, {hover_msg.pose.position.y}, {hover_msg.pose.position.z}) for {duration} seconds")
        logging.info(f"Published emergency hover command at position: ({hover_msg.pose.position.x}, {hover_msg.pose.position.y}, {hover_msg.pose.position.z}) for {duration} seconds")

        rospy.signal_shutdown("Emergency hover command sent")

    rospy.Subscriber(f'/{prefix}/mavros/local_position/odom', Odometry, current_position_cb)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        logging.error("Usage: python send_emergency_hover_command.py <prefix> <duration>")
        sys.exit(1)

    prefix = sys.argv[1]
    duration = float(sys.argv[2])
    send_emergency_hover_command(prefix, duration)

