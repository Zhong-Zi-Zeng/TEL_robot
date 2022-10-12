#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import rospy

class ImageCallback:
    def __init__(self):
        # 初始化節點
        self.bridge = CvBridge()
        rospy.init_node('subscribe_image')

    # 返回RGB影像
    def get_img(self):
        rgb_img = rospy.wait_for_message('/camera/color/image_raw', Image)
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_img, 'bgr8')

        return rgb_img

    # 返回Depth影像
    def get_depth_img(self):
        depth_img = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)
        depth_img = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")
        depth_img = np.float16(depth_img) / 10

        return depth_img


