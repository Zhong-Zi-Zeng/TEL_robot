#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Uart_Api import UartApi
import rospy

class Level3:
    def __init__(self, img_queue):
        # 影像監聽
        self.img_queue = img_queue

        # 初始化Uart Api
        self.uart_api = UartApi()

        # 載入參數
        self.debug = rospy.get_param('/Debug')  # Debug模式

    def start(self, level2_finish):
        # 判斷第一關是否有完成，沒有則傳送從第二關重新開始的命令
        if not level2_finish:
            if self.debug:
                print('Level3 Restart')
            self.uart_api.send_special_order('l')

        if self.debug:
            print('Level3 Start')
