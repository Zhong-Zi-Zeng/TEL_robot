#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from std_msgs.msg import String
from Uart_Api import UartApi
import rospy

def callback(data_msg):
    data = data_msg.data
    data = data.split(',')

    direction = data[0]
    speed = data[1]
    motor0_base = data[2]
    motor1_base = data[3]

    uart_api.send_order(direction=direction, value=speed, motor_1=motor0_base, motor_2=motor1_base)

if __name__ == '__main__':
    # 初始化
    rospy.init_node("keyboard_subscriber")
    rospy.Subscriber('keyboard_topic', String, callback)
    rospy.loginfo('Start keyboard subscriber...')

    uart_api = UartApi()

    # Debug
    debug = rospy.get_param('/Debug')

    rospy.spin()
