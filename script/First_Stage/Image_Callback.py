#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from collections import deque
import numpy as np
import cv2
import rospy

class ImageCallback:
    def __init__(self):
        # 建立柱列
        self.img_queue = deque(maxlen=1)
        self.depth_img_queue = deque(maxlen=1)

        # 初始化節點
        self.bridge = CvBridge()
        rospy.init_node('subscribe_image')
        rospy.Subscriber('/camera/color/image_raw', Image, self._img_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._depth_img_callback)
        rospy.spin()

    # RGB影像回調函式
    def _img_callback(self, img_msgs):
        img = self.bridge.imgmsg_to_cv2(img_msgs, 'bgr8')

        self.img_queue.append(img)

    # 深度影像回調函式
    def _depth_img_callback(self, depth_img_msgs):
        depth_img = self.bridge.imgmsg_to_cv2(depth_img_msgs, "32FC1")

        self.depth_img_queue.append(depth_img * 0.001)

    def get_img(self):
        while len(self.img_queue) == 0:
            pass

        return self.img_queue.pop()

    def get_depth_img(self):
        while len(self.depth_img_queue) == 0:
            pass

        return self.depth_img_queue.pop()

