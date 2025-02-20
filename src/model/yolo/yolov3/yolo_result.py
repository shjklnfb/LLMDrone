import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from datetime import datetime
from picture_gpt import get_image_description
import time
# 全局变量来存储最新的图像
latest_img = None

def image_callback(msg):
    global latest_img
    height = msg.height
    width = msg.width
    channels = msg.step // width
    latest_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)
    cv2.imshow('Annotated Image', latest_img)
    cv2.waitKey(1)

def save_image(event):
    global latest_img
    if latest_img is not None:
        # 生成基于时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"image_{timestamp}.jpg"
        cv2.imwrite(filename, latest_img)
        time.sleep(1)
        print(get_image_description(filename))
        rospy.loginfo(f"Image saved to {filename}")

# 初始化ROS节点
rospy.init_node('image_viewer')

# 设置定时器，每10秒（10000毫秒）调用一次save_image函数
timer = rospy.Timer(rospy.Duration(10), save_image)

# 订阅图像话题
image_sub = rospy.Subscriber('/annotated_image', Image, image_callback)

# 进入ROS事件循环
rospy.spin()
