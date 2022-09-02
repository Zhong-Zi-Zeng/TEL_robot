#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from pynput.keyboard import Key
from pynput.keyboard import Listener
from std_msgs.msg import String

class KeyboardManger:
    def __init__(self):
        self.KEYBOARD_DICT = self._initial_keyboard_dict()

    def _initial_keyboard_dict(self):

        keyboard_dict = {
            # 前
            'w': {'code': rospy.get_param('/Keyboard/up/code'),
                  'value': rospy.get_param('/Keyboard/up/value')},
            # 後
            's': {'code': rospy.get_param('/Keyboard/down/code'),
                  'value': rospy.get_param('/Keyboard/down/value')},
            # 平移左
            'a': {'code': rospy.get_param('/Keyboard/left/code'),
                  'value': rospy.get_param('/Keyboard/left/value')},
            # 平移右
            'd': {'code': rospy.get_param('/Keyboard/right/code'),
                  'value': rospy.get_param('/Keyboard/right/value')},
            # 順時針轉
            'e': {'code': rospy.get_param('/Keyboard/p_circle/code'),
                  'value': rospy.get_param('/Keyboard/p_circle/value')},
            # 逆時針
            'q': {'code': rospy.get_param('/Keyboard/n_circle/code'),
                  'value': rospy.get_param('/Keyboard/n_circle/value')},
        }

        return keyboard_dict

    # ======當鍵盤被按下時======
    def keyboard_press(self, key):
        try:
            # 按下一般按鍵
            if key.char in self.KEYBOARD_DICT.keys():
                self._send_keyboard(code=self.KEYBOARD_DICT[key.char]['code'],
                                    value=self.KEYBOARD_DICT[key.char]['value'])

        # 按下一般特殊按鍵
        except AttributeError:
            if key in self.KEYBOARD_DICT.keys():
                self._send_keyboard(code=self.KEYBOARD_DICT[key.char]['code'],
                                    value=self.KEYBOARD_DICT[key.char]['value'])
            # 按下esc鍵
            elif key == Key.esc:
                rospy.loginfo('Press Esc')
                return False

    # ======發布話題======
    def _send_keyboard(self, code: str, value: int):
        order = code + str(value)

        pub.publish(order)

if __name__ == '__main__':
    # 初始化
    rospy.init_node("keyboard_publisher")
    pub = rospy.Publisher('keyboard_topic', String, queue_size=1)
    keyboard_manger = KeyboardManger()

    # 監聽鍵盤
    rospy.loginfo('Start listener keyboard...')
    rospy.loginfo('Press ESC exit.')
    with Listener(on_press=keyboard_manger.keyboard_press) as listener:
        listener.join()
