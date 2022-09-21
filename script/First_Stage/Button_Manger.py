#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import RPi.GPIO as GPIO
import rospy

class ButtonManger:
    def __init__(self):
        # 初始化接腳
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self.level1_pin = rospy.get_param('/Button/Level1')
        self.level2_pin = rospy.get_param('/Button/Level2')
        self.level3_pin = rospy.get_param('/Button/Level3')

        self.set_pin()

    # ===========設置接腳===========
    def set_pin(self):
        GPIO.setup(self.level1_pin, GPIO.IN)
        GPIO.setup(self.level2_pin, GPIO.IN)
        GPIO.setup(self.level3_pin, GPIO.IN)

    # ===========讀取第一關接腳===========
    def read_level1_start(self):
        return GPIO.input(self.level1_pin)

    # ===========讀取第二關接腳===========
    def read_level2_start(self):
        return GPIO.input(self.level2_pin)

    # ===========讀取第三關接腳===========
    def read_level3_start(self):
        return GPIO.input(self.level3_pin)
