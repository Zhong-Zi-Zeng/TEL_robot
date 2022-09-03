#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Button_Manger import ButtonManger
from Image_Callback import ImageCallback
from Level1 import Level1
from Level2 import Level2
from Level3 import Level3
import rospy


class LevelManager:
    def __init__(self):
        # 初始化開關管理員
        self.button_manger = ButtonManger()

        # 初始化影像佇列物件
        self.img_queue = ImageCallback()

        # Test mode
        self.test_mode = rospy.get_param('/TestMode')
        self.test_level = rospy.get_param('/TestLevel')

        # 關卡紀錄
        self.level1_finish = False
        self.level2_finish = False
        self.level3_finish = False

        # 初始化各關卡
        self.level1 = Level1(img_queue=self.img_queue)
        self.level2 = Level2(img_queue=self.img_queue)
        self.level3 = Level3(img_queue=self.img_queue)

        # 判斷是否是測試模式
        if not self.test_mode:
            self.listen_button()
        elif self.test_level == 1:
            self.level1.start()
        elif self.test_level == 2:
            self.level2.start()
        else:
            self.level3.start()


    def listen_button(self):
        while not rospy.is_shutdown():
            # 判斷關卡按鈕有沒有被按下
            if self.button_manger.read_level1_start():
                if self.level1_finish == False:
                    self.level1_finish = True if self.level1.start() else False

            elif self.button_manger.read_level2_start() or self.level1_finish:
                if self.level2_finish == False:
                    self.level2_finish = True if self.level2.start() else False

            elif self.button_manger.read_level3_start() or self.level2_finish:
                if self.level3_finish == False:
                    self.level3_finish = True if self.level3.start() else False




if __name__ == '__main__':
    LevelManager()
