#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy

class Debug:
    def __init__(self):
        self.debug = rospy.get_param('/Debug')  # Debug模式

    def debug_info(self, *args):
        if not self.debug:
            return

        for arg in args:
            print arg,
        print
