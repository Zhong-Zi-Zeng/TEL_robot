#!/usr/bin/env python
from Yolo_V4_Api import Detect
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rospy


# =====影像回調函式=====
def img_callback(img_msgs):
    img = bridge.imgmsg_to_cv2(img_msgs, 'bgr8')
    detections = network.detect_image(img)

    print(detections)

    cv2.imshow('', img)
    cv2.waitKey(1)




if __name__ == '__main__':
    network = Detect()

    # 訂閱影像節點
    bridge = CvBridge()
    subscriber = rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
    rospy.spin()







