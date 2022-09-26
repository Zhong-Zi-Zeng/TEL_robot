#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import serial
import rospy

class ButtonManger:
    def __init__(self):
        # 初始化Serial
        self.port = rospy.get_param('/MegaPort')
        self.baudrate = rospy.get_param('/MegaBaudrate')
        self.ser = serial.Serial(self.port, self.baudrate)

    def read_response(self, level):
        self.ser.write(list(map(ord, level)))

        while self.ser.in_waiting:
            response = str(self.ser.read().decode('utf-8'))
            print(response)
            return response

    # ===========讀取第一關接腳===========
    def read_level1_start(self):
        return self.read_response('1')

    # ===========讀取第二關接腳===========
    def read_level2_start(self):
        return self.read_response('2')

    # ===========讀取第三關接腳===========
    def read_level3_start(self):
        return self.read_response('3')
