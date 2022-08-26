#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from Yolo_V4_Api import Detect
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rospy


# =====Image callback function=====
def img_callback(img_msgs):
    img = bridge.imgmsg_to_cv2(img_msgs, 'bgr8')
    detections = network.detect_image(img)

    print(detections)

    cv2.imshow('', img)
    cv2.waitKey(1)




if __name__ == '__main__':
    network = Detect()
    rospy.init_node('main')

    # 訂閱影像節點
    bridge = CvBridge()
    subscriber = rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
    rospy.spin()







