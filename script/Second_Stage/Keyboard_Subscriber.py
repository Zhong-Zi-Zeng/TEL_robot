#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String


def callback(data):
    print(data)


if __name__ == '__main__':
    rospy.init_node("keyboard_subscriber")
    pub = rospy.Subscriber('keyboard_topic', String, callback)

    # 初始化Serial
    port = rospy.get_param('/port')
    baudrate = rospy.get_param('/baudrate')
    print(port, baudrate)

    

