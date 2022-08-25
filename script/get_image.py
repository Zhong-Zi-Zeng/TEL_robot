#!/usr/bin/env python

import rospy
import cv2
# import pygame
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


def show(img_msgs):
    img = bridge.imgmsg_to_cv2(img_msgs, 'bgr8')

    cv2.imshow('', img)
    cv2.waitKey(1)

def pub_key():
    while True:
        pass


if __name__ == '__main__':
    rospy.init_node('image_subscriber')

    bridge = CvBridge()
    subscriber = rospy.Subscriber('image_topic', Image, show)
    # publisher = rospy.Publisher('keyboard_topic', String, queue_size=1)
    pub_key()
    # rospy.spin()
