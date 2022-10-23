#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import serial
import rospy


class UartApi:
    def __init__(self):
        # debug
        self.test_mode = rospy.get_param('/TestMode')

        # 初始化Serial
        self.port = rospy.get_param('/MegaPort')
        self.baudrate = rospy.get_param('/MegaBaudrate')
        self.ser = serial.Serial(self.port, self.baudrate)

    # ======傳送一般指令======
    def send_order(self, direction='p', value='0', degree='0', motor_1='115', motor_2='110'):
        assert isinstance(direction, str), 'Argument direction type is not str'
        assert isinstance(value, str), 'Argument  value type is not str'
        assert isinstance(degree, str), 'Argument degree type is not str'
        assert isinstance(motor_1, str), 'Argument motor_1 type is not str'
        assert isinstance(motor_2, str), 'Argument motor_2 type is not str'

        value = value.zfill(3)
        degree = degree.zfill(3)
        motor_1 = motor_1.zfill(3)
        motor_2 = motor_2.zfill(3)

        order = '[' + direction + value + degree + motor_1 + motor_2
        order = list(map(ord, order))

        self.ser.write(order)

    # ======傳送特殊指令======
    def send_special_order(self, action):
        assert isinstance(action, str), 'Argument value type not str'

        order = '<' + action
        order = list(map(ord, order))
        self.ser.write(order)

        if self.test_mode:
            return True
        else:
            while True:
                while self.ser.in_waiting:
                    response = str(self.ser.read().decode('utf-8'))

                    if response == 'a':
                        return True
                    else:
                        return False
