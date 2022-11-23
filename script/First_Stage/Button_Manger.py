#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import serial
import rospy
import time


class ButtonManger:
    def __init__(self):
        # 初始化Serial
        self.port = rospy.get_param('/NanoPort')
        self.baudrate = rospy.get_param('/NanoBaudrate')
        self.ser = serial.Serial(self.port, self.baudrate)

    def read_response(self, level):
        self.ser.write(level)

        while True:
            while self.ser.in_waiting:
                data = int(self.ser.read().decode('utf-8'))
                if data:
                    return True
                return False

    # ===========讀取第一關接腳===========
    def read_level1_start(self):
        return self.read_response('1')

    # ===========讀取第二關接腳===========
    def read_level2_start(self):
        return self.read_response('2')

    # ===========讀取第三關接腳===========
    def read_level3_start(self):
        return self.read_response('3')
