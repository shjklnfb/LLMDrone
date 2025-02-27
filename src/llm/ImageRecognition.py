from openai import OpenAI
import os
import base64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
import base64
import cv2
from cv_bridge import CvBridge
from openai import OpenAI
from sensor_msgs.msg import Image

# 将 ROS 的 sensor_msgs/Image 转换为 Base64 编码的图像数据
def image_msg_to_base64(image_msg):
    bridge = CvBridge()
    # 将 ROS 图像消息转换为 OpenCV 格式
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    # 将 OpenCV 图像编码为 JPEG 格式
    _, buffer = cv2.imencode('.jpg', cv_image)
    # 将图像数据转换为 Base64 编码
    base64_image = base64.b64encode(buffer).decode("utf-8")
    return base64_image

# 图像分析方法
def analyze_image(image_msg):
    # 将 ROS 图像消息转换为 Base64 编码
    base64_image = image_msg_to_base64(image_msg)

    # 初始化 OpenAI 客户端
    client = OpenAI(
        api_key=os.getenv("OPENAI_API_KEY", "sk-d34cba22d2a04a5c8c191f082106d07e"),
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
    )

    # 调用 OpenAI API 进行图像分析
    completion = client.chat.completions.create(
        model="qwen-vl-plus",
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
                    },
                    {"type": "text", "text": "这个图片是一架在仿真环境中飞行的无人机携带的底部摄像头拍摄的，你需要识别出当前图片中的内容，描述具体信息"}
                ]
            }
        ]
    )
    return completion.choices[0].message.content

