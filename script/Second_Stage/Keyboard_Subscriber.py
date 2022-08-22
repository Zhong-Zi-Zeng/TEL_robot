#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

def callback(data):
    if debug:
        print(data)


if __name__ == '__main__':
    rospy.init_node("keyboard_subscriber")
    pub = rospy.Subscriber('keyboard_topic', String, callback)
    rospy.loginfo('Start keyboard subscriber...')

    # Debug
    debug = rospy.get_param('/debug')

    # Init Serial
    port = rospy.get_param('/port')
    baudrate = rospy.get_param('/baudrate')
    # ser = serial.Serial(port, baudrate)
    rospy.spin()
