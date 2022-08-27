#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from std_msgs.msg import String
from Uart_Api import UartApi
import rospy

def callback(data):
    data = str(data)
    code = data[0]
    value = data[1:]

    uart_api.send_order(direction=code, value=value)

if __name__ == '__main__':
    # 初始化
    rospy.init_node("keyboard_subscriber")
    pub = rospy.Subscriber('keyboard_topic', String, callback)
    rospy.loginfo('Start keyboard subscriber...')

    uart_api = UartApi()


    # Debug
    debug = rospy.get_param('/debug')

    rospy.spin()
