#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def show(img_msgs):
    img = bridge.imgmsg_to_cv2(img_msgs, 'bgr8')
    cv2.imshow('', img)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_subscriber')
    bridge = CvBridge()
    rospy.Subscriber('/camera/color/image_raw', Image, show)
    rospy.loginfo("Start subscriber camera...")
    rospy.spin()
