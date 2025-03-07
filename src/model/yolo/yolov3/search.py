import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import rospy
from sensor_msgs.msg import Image

# 加载YOLOv3 Tiny模型
cfg_path = os.path.expanduser("~/Desktop/LLMDrone/src/model/yolo/yolov3/yolov3-tiny.cfg")
weights_path = os.path.expanduser("~/Desktop/LLMDrone/src/model/yolo/yolov3/yolov3-tiny.weights")
coco_path = os.path.expanduser("~/Desktop/LLMDrone/src/model/yolo/yolov3/coco.names")

net = cv2.dnn.readNet(weights_path, cfg_path)
with open(coco_path, "r") as f:
    classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
unconnected_out_layers = net.getUnconnectedOutLayers().flatten()
output_layers = [layer_names[p - 1] for p in unconnected_out_layers]

# ROS桥接
bridge = CvBridge()
# rospy.init_node('yolo_detector', anonymous=True)
video_pub = rospy.Publisher('/detection_video', Image, queue_size=10)

def detect_target(image_msg, target_label):
    # 将ROS图像消息转换为OpenCV图像
    try:
        img = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    except Exception as e:
        print(f"Error converting ROS image to OpenCV image: {e}")
        return False, None

    height, width = img.shape[:2]

    # YOLO目标检测预处理
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # 存储检测结果
    class_ids = []
    confidences = []
    boxes = []
    target_position = None  # 初始化目标位置变量

    # 解析YOLOv3的输出
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # 只选择置信度大于0.5的检测结果
                # 反归一化中心点坐标和宽高
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # 计算左上角坐标
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                # 确保坐标在图像范围内
                x = max(0, min(x, width - 1))
                y = max(0, min(y, height - 1))
                w = max(0, min(w, width - x))
                h = max(0, min(h, height - y))

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)


    # 应用非最大抑制（NMS）去除冗余框
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    if len(indexes) > 0:
        for i in indexes.flatten():
            # 获取标注框坐标
            x, y, w, h = boxes[i]

            # 绘制检测框
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, classes[class_ids[i]], (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            if classes[class_ids[i]] == target_label:
                target_position = boxes[i]  # 保存目标标签的位置

                # 保存带有标注框的图像到本地
                cv2.imwrite("output_with_box.jpg", img)

                # 发布带有标注框的图像
                try:
                    video_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
                except Exception as e:
                    print(f"Error converting OpenCV image to ROS image: {e}")

                return True, target_position  # 返回目标标签的位置

    # 发布未检测到目标标签的图像
    try:
        video_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    except Exception as e:
        print(f"Error converting OpenCV image to ROS image: {e}")
    
    return False, target_position  # 如果未检测到目标标签，返回None