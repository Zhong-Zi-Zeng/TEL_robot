#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy

class Debug:
    def __init__(self):
        self.debug = rospy.get_param('/Debug')  # Debug模式

    def debug_info(self, info, *args):
        if not self.debug:
            return

        print(info)
        for arg in args:
            print(arg, end=" ")
        print()

