import rospy
import sys
import logging
from datetime import datetime
from geometry_msgs.msg import PoseStamped

# 设置日志记录
log_filename = datetime.now().strftime('../log/flight/command_pos_%Y%m%d_%H%M%S.log')
logging.basicConfig(filename=log_filename, level=logging.INFO)

def publish_command(prefix, x, y, z):
    node_name = f'{prefix}_command_publisher'
    # rospy.init_node(node_name, anonymous=True,)
    command_topic = f'/{prefix}/drone/command'
    command_pub = rospy.Publisher(command_topic, PoseStamped, queue_size=100)

    rospy.sleep(1) 
    # 创建一个 PoseStamped 消息
    command = PoseStamped()
    command.pose.position.x = x
    command.pose.position.y = y
    command.pose.position.z = z
    command.header.stamp = rospy.Time.now()
    rospy.loginfo(f"Publishing new command: ({x}, {y}, {z})")
    logging.info(f"Publishing new command: ({x}, {y}, {z})")
    command_pub.publish(command)

    rospy.loginfo("Command published")
    logging.info("Command published")

if __name__ == '__main__':
    try:
        # 获取命令行参数
        if len(sys.argv) != 5:
            logging.error("Usage: pos.py prefix x y z")
            sys.exit(1)

        prefix = sys.argv[1]
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        z = float(sys.argv[4])
        rospy.sleep(1)  # 确保已经启动了 ROS 节点
        publish_command(prefix, x, y, z)  # 发布新指令，飞到指定的 (x, y, z) 位置

    except rospy.ROSInterruptException:
        pass

