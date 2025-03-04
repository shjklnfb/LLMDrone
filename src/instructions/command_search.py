import rospy
import sys
import logging
from datetime import datetime
from geometry_msgs.msg import PoseStamped

# 设置日志记录
log_filename = datetime.now().strftime('../log/flight/command_random_%Y%m%d_%H%M%S.log')
logging.basicConfig(filename=log_filename, level=logging.INFO)

def publish_random_flight_command(prefix, radius, duration):
    node_name = f'{prefix}_random_flight_command_publisher'
    # rospy.init_node(node_name, anonymous=True,)
    command_topic = f'/{prefix}/drone/random_flight_command'
    command_pub = rospy.Publisher(command_topic, PoseStamped, queue_size=100)

    rospy.sleep(1) 
    # 创建一个 PoseStamped 消息
    command = PoseStamped()
    command.pose.position.x = radius  # 半径
    command.pose.position.y = duration  # 持续时间
    command.header.stamp = rospy.Time.now()
    rospy.loginfo(f"Publishing random flight command: radius={radius}, duration={duration}")
    logging.info(f"Publishing random flight command: radius={radius}, duration={duration}")
    command_pub.publish(command)

    rospy.loginfo("Random flight command published")

if __name__ == '__main__':
    try:
        # 获取命令行参数
        if len(sys.argv) != 4:
            logging.error("Usage: random_flight.py prefix radius duration")
            sys.exit(1)

        prefix = sys.argv[1]
        radius = float(sys.argv[2])
        duration = float(sys.argv[3])
        
        publish_random_flight_command(prefix, radius, duration)  # 发布随机飞行指令

    except rospy.ROSInterruptException:
        pass