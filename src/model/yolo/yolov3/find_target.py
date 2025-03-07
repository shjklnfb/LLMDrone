#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import os
# # 初始化ROS节点
# rospy.init_node('offboard_ctr')

# 加载YOLOv3 Tiny模型
cfg_path = os.path.expanduser("~/Desktop/LLMDrone/src/model/yolo/yolov3/yolov3-tiny.cfg")
weights_path = os.path.expanduser("~/Desktop/LLMDrone/src/model/yolo/yolov3/yolov3-tiny.weights")
coco_path = os.path.expanduser("~/Desktop/LLMDrone/src/model/yolo/yolov3/coco.names")

net = cv2.dnn.readNet(weights_path, cfg_path)
with open(coco_path, "r") as f:
    classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
unconnected_out_layers = net.getUnconnectedOutLayers()

# 确保unconnected_out_layers中的每个元素都是整数
output_layers = [layer_names[p[0] - 1] for p in unconnected_out_layers]

# ROS桥接
bridge = CvBridge()

# 定义目标识别的回调函数
def image_callback(msg):
    # 将ROS图像消息转换为OpenCV图像
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Error converting ROS image to OpenCV image: {e}")
        return

    height, width, channels = img.shape

    # YOLO目标检测预处理
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # 存储检测结果
    class_ids = []
    confidences = []
    boxes = []

    # 解析YOLOv3的输出
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # 只选择信心度大于0.5的检测结果
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # 应用非最大抑制（NMS）去除冗余框
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    # 在图像上绘制目标框和标签
    if len(indexes) > 0:
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            # 绘制矩形框
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # 添加标签
            cv2.putText(img, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 将处理后的图像转换为ROS消息并发布
    annotated_image_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    annotated_image_pub.publish(annotated_image_msg)

    # 可选：将检测结果输出到控制台或其他地方
    result_str = str([{"class": classes[class_ids[i]], "confidence": confidences[i], "box": boxes[i]} for i in indexes.flatten()])
    result_pub.publish(result_str)

# 图像订阅
image_sub = rospy.Subscriber('/iris_0/realsense/depth_camera/color/image_raw', Image, image_callback)

# 标注图像的发布
annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=10)

# 检测结果发布
result_pub = rospy.Publisher('/detection_results', String, queue_size=10)

# rospy.spin()

