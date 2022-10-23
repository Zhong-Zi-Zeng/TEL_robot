#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from std_msgs.msg import String
from Uart_Api import UartApi
import rospy


def callback(data_msg):
    data = data_msg.data
    data = data.split(',')

    if len(data) == 4:
        direction = data[0]
        speed = data[1]
        motor0_base = data[2]
        motor1_base = data[3]

        uart_api.send_order(direction=direction, value=speed, motor_1=motor0_base, motor_2=motor1_base)
    else:
        uart_api.send_special_order(data[0])


if __name__ == '__main__':
    # 初始化
    rospy.init_node("keyboard_subscriber")
    rospy.Subscriber('keyboard_topic', String, callback)
    rospy.loginfo('Start keyboard subscriber...')

    uart_api = UartApi()
    uart_api.send_special_order(action='s')  # 關閉回正
    # Debug
    debug = rospy.get_param('/Debug')

    rospy.spin()
