#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Button_Manger import ButtonManger
from Image_Callback import ImageCallback
from Level1 import Level1
from Level2 import Level2
from Level3 import Level3
from Uart_Api import UartApi
from Debug import Debug
import time
import rospy


class LevelManager:
    def __init__(self):
        # 初始化開關管理員
        self.button_manger = ButtonManger()

        # 初始化影像佇列物件
        self.img_queue = ImageCallback()

        # uart
        self.uart_api = UartApi()

        # Debug
        self.debug = Debug()

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
            self.level2.start(self.level1_finish)
        else:
            self.level3.start(self.level2_finish)

    def listen_button(self):
        while not rospy.is_shutdown():
            # 判斷關卡按鈕有沒有被按下
            if self.button_manger.read_level1_start():
                self.debug.debug_info("Wait...")
                time.sleep(3)
                if self.button_manger.read_level2_start():
                    self.debug.debug_info("To Level2")
                    self.uart_api.send_special_order(action='j')
                elif not self.level1_finish:
                    self.level1_finish = True if self.level1.start() else False

            if self.button_manger.read_level2_start() or self.level1_finish:
                if not self.level2_finish:
                    self.level2_finish = True if self.level2.start(self.level1_finish) else False

            if self.button_manger.read_level3_start() or self.level2_finish:
                if not self.level3_finish:
                    self.level3_finish = True if self.level3.start(self.level2_finish) else False

if __name__ == '__main__':
    LevelManager()
