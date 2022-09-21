#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
from threading import Thread
from pynput.keyboard import Key
from pynput.keyboard import Listener
from std_msgs.msg import String


class KeyboardManger:
    def __init__(self):
        self.KEYBOARD_DICT = self._initial_keyboard_dict()
        self.motor0_base = rospy.get_param('/Keyboard/motor0_base')
        self.motor1_base = rospy.get_param('/Keyboard/motor1_base')
        self.direction = 'p'
        self.speed = '0'
        print(self.motor0_base, self.motor1_base)
        # 利用子執行序不斷發布消息
        Thread(target=self._send_keyboard).start()

    def _initial_keyboard_dict(self):

        keyboard_dict = {
            # 前
            'w': {'direction': rospy.get_param('/Keyboard/up/direction'),
                  'speed': rospy.get_param('/Keyboard/up/speed')},
            # 後
            's': {'direction': rospy.get_param('/Keyboard/down/direction'),
                  'speed': rospy.get_param('/Keyboard/down/speed')},
            # 平移左
            'a': {'direction': rospy.get_param('/Keyboard/left/direction'),
                  'speed': rospy.get_param('/Keyboard/left/speed')},
            # 平移右
            'd': {'direction': rospy.get_param('/Keyboard/right/direction'),
                  'speed': rospy.get_param('/Keyboard/right/speed')},
            # 順時針轉
            'e': {'direction': rospy.get_param('/Keyboard/p_circle/direction'),
                  'speed': rospy.get_param('/Keyboard/p_circle/speed')},
            # 逆時針
            'q': {'direction': rospy.get_param('/Keyboard/n_circle/direction'),
                  'speed': rospy.get_param('/Keyboard/n_circle/speed')},
        }

        return keyboard_dict

    # ======當鍵盤被按下時======
    def keyboard_press(self, key):
        try:
            # 按下一般按鍵
            if key.char in self.KEYBOARD_DICT.keys():
                self.direction = self.KEYBOARD_DICT[key.char]['direction']
                self.speed = self.KEYBOARD_DICT[key.char]['speed']

        # 按下一般特殊按鍵
        except AttributeError:
            if key == Key.up:
                self.motor1_base += 1
            if key == Key.down:
                self.motor1_base -= 1
            if key == Key.left:
                self.motor0_base += 1
            if key == Key.right:
                self.motor0_base -= 1

            # 按下esc鍵
            if key == Key.esc:
                rospy.loginfo('Press Esc')
                return False
        except:
            pass

    # ======當鍵盤被放開時======
    def keyboard_release(self, key):
        try:
            if key.char in self.KEYBOARD_DICT.keys():
                self.direction = 'p'
                self.speed = 0
        except:
            pass

    # ======發布話題======
    def _send_keyboard(self):
        while not rospy.is_shutdown():
            order = [self.direction, str(self.speed), str(self.motor0_base), str(self.motor1_base)]
            pub.publish(','.join(order))

if __name__ == '__main__':
    # 初始化
    rospy.init_node("keyboard_publisher")
    pub = rospy.Publisher('keyboard_topic', String, queue_size=1)
    keyboard_manger = KeyboardManger()

    # 監聽鍵盤
    rospy.loginfo('Start listener keyboard...')
    rospy.loginfo('Press ESC exit.')
    with Listener(on_press=keyboard_manger.keyboard_press, on_release=keyboard_manger.keyboard_release) as listener:
        listener.join()
