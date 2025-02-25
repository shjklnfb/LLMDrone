#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
import sys
import logging
from datetime import datetime

# 设置日志记录
log_filename = datetime.now().strftime('../log/flight/command_land_%Y%m%d_%H%M%S.log')
logging.basicConfig(filename=log_filename, level=logging.INFO)

def send_land_command(prefix):
    node_name = f'{prefix}_land_command_sender'
    # rospy.init_node(node_name, anonymous=True)
    land_topic = f'/{prefix}/drone/land_command'
    land_pub = rospy.Publisher(land_topic, PoseStamped, queue_size=10)
    rospy.sleep(1)

    # 创建一个空的 PoseStamped 消息来表示降落指令
    land_msg = PoseStamped()
    land_msg.pose.position.x = 0
    land_msg.pose.position.y = 0
    land_msg.pose.position.z = 0  # 降落到地面

    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo("Publishing land command")
    logging.info("Publishing land command")
    land_pub.publish(land_msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        # 获取命令行参数
        if len(sys.argv) != 2:
            logging.error("Usage: python send_land_command.py <prefix>")
            sys.exit(1)

        prefix = sys.argv[1]

        send_land_command(prefix)
    except rospy.ROSInterruptException:
        pass
