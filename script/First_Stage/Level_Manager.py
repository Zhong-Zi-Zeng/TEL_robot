#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Button_Manger import ButtonManger
from Level1 import Level1
from Level2 import Level2
from Level3 import Level3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rospy


class LevelManager:
    def __init__(self):
        # 初始化開關管理員
        self.button_manger = ButtonManger()

        # 關卡紀錄
        self.level1_finish = False
        self.level2_finish = False
        self.level3_finish = False

        # 初始化各關卡
        self.level1 = Level1()
        self.level2 = Level2()
        self.level3 = Level3()

        # 訂閱影像節點
        self.bridge = CvBridge()
        rospy.init_node('main')
        rospy.Subscriber('/camera/color/image_raw', Image, self.img_callback)
        rospy.spin()

    # =====Image callback function=====
    def img_callback(self, img_msgs):
        img = self.bridge.imgmsg_to_cv2(img_msgs, 'bgr8')

        # 判斷關卡按鈕有沒有被按下
        if self.button_manger.read_level1_start():
            if self.level1_finish == False:
                self.level1_finish = True if self.level1.start(img) else False

        elif self.button_manger.read_level2_start() or self.level1_finish:
            if self.level2_finish == False:
                self.level2_finish = True if self.level2.start(img) else False

        elif self.button_manger.read_level3_start() or self.level2_finish:
            if self.level3_finish == False:
                self.level3_finish = True if self.level3.start(img) else False

        cv2.imshow('', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    LevelManager()
