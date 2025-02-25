import rospy
import sys
import logging
from datetime import datetime
from geometry_msgs.msg import TwistStamped
import math

# 设置日志记录
log_filename = datetime.now().strftime('../log/flight/command_twist_%Y%m%d_%H%M%S.log')
logging.basicConfig(filename=log_filename, level=logging.INFO)

def send_rotate_command(prefix, yaw_rate, duration):
    node_name = f'{prefix}_rotate_command_sender'
    # rospy.init_node(node_name, anonymous=True,)
    rotate_pub = rospy.Publisher(f'/{prefix}/drone/rotate_command', TwistStamped, queue_size=10)
    rospy.sleep(1)

    rotate_msg = TwistStamped()
    rotate_msg.twist.angular.z = math.radians(yaw_rate)  # 设置旋转速率
    rotate_msg.twist.linear.x = duration
    rate = rospy.Rate(10)  # 10 Hz
    rotate_pub.publish(rotate_msg)
    rate.sleep()

    rospy.loginfo(f"Published rotate command with yaw_rate: {yaw_rate} degrees/sec for {duration} seconds")
    logging.info(f"Published rotate command with yaw_rate: {yaw_rate} degrees/sec for {duration} seconds")


if __name__ == '__main__':
    # 获取命令行参数
    if len(sys.argv) != 4:
        logging.error("Usage: python send_rotate_command.py <prefix> <yaw_rate> <duration>")
        sys.exit(1)

    prefix = sys.argv[1]
    yaw_rate = float(sys.argv[2])
    duration = float(sys.argv[3])
    send_rotate_command(prefix, yaw_rate, duration)