#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
import logging
from datetime import datetime
from std_msgs.msg import Bool

# 设置日志记录
log_filename = datetime.now().strftime('../log/flight/command_interrupt_%Y%m%d_%H%M%S.log')
logging.basicConfig(filename=log_filename, level=logging.INFO)

def send_interrupt_command(prefix):
    node_name = f'{prefix}_interrupt_command_sender'
    rospy.init_node(node_name, anonymous=True)
    interrupt_topic = f'/{prefix}/drone/interrupt_command'
    interrupt_pub = rospy.Publisher(interrupt_topic, Bool, queue_size=10)
    rospy.sleep(1)

    interrupt_msg = Bool()
    interrupt_msg.data = True

    rospy.loginfo("Publishing interrupt command")
    logging.info("Publishing interrupt command")
    interrupt_pub.publish(interrupt_msg)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        logging.error("Usage: python send_interrupt_command.py <prefix>")
        sys.exit(1)

    prefix = sys.argv[1]
    send_interrupt_command(prefix)
