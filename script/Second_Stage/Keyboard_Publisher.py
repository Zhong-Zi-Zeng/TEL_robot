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
        self.NORMAL_KEYBOARD_DICT, self.SPECIAL_KEYBOARD_DICT = self._initial_keyboard_dict()
        self.motor0_degree = rospy.get_param('/Keyboard/motor0_base')
        self.motor1_degree = rospy.get_param('/Keyboard/motor1_base')
        self.direction = 'p'
        self.speed = '0'

        # 上下左右鍵狀態
        self.dir_keyboard_state = {
            'up': False,
            'down': False,
            'left': False,
            'right': False
        }

        # 利用子執行序不斷發布消息
        Thread(target=self._send_keyboard).start()

    def _initial_keyboard_dict(self):
        normal_keyboard_dict = {
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

        special_keyboard_dict = {
            # 放下方塊
            'r': self.position_arm,
            # 吸取方塊
            'f': self.suck_cube,
            # 釋放方塊
            'g': self.release_cube,
            # 開啟閘道
            'v': self.open_gate,
            # 關閉閘道
            'b': self.close_gate,
        }

        return normal_keyboard_dict, special_keyboard_dict

    # ======當鍵盤被按下時======
    def keyboard_press(self, key):
        try:
            # 按下一般按鍵
            if key.char in self.NORMAL_KEYBOARD_DICT.keys():
                self.direction = self.NORMAL_KEYBOARD_DICT[key.char]['direction']
                self.speed = self.NORMAL_KEYBOARD_DICT[key.char]['speed']

            # 按下功能按鍵
            if key.char in self.SPECIAL_KEYBOARD_DICT.keys():
                self.SPECIAL_KEYBOARD_DICT[key.char]()

        # 按下特殊按鍵
        except AttributeError:
            if key == Key.up:
                self.dir_keyboard_state['up'] = True
            if key == Key.down:
                self.dir_keyboard_state['down'] = True
            if key == Key.left:
                self.dir_keyboard_state['left'] = True
            if key == Key.right:
                self.dir_keyboard_state['right'] = True

            # 按下esc鍵
            if key == Key.esc:
                rospy.loginfo('Press Esc')
                return False
        except:
            pass

    # ======吸取======
    def suck_cube(self):
        pub.publish('a')

    # ======釋放======
    def release_cube(self):
        pub.publish('b')

    # ======開啟閘道======
    def open_gate(self):
        pub.publish('c')

    # ======關閉閘道======
    def close_gate(self):
        pub.publish('d')

    # ======控制手臂======
    def control_arm(self):
        if self.dir_keyboard_state['up'] and self.motor1_degree > 0:
            self.motor1_degree -= 1

        if self.dir_keyboard_state['down'] and self.motor1_degree < 180:
            self.motor1_degree += 1

        if self.dir_keyboard_state['left'] and self.motor0_degree < 180:
            self.motor0_degree += 1

        if self.dir_keyboard_state['right'] and self.motor0_degree > 0:
            self.motor0_degree -= 1

    # ======定位手臂到放料位置======
    def position_arm(self):
        motor0_base = rospy.get_param('/Keyboard/putdown_cube/motor0')
        motor1_base = rospy.get_param('/Keyboard/putdown_cube/motor1')

        # 調整motor1
        while self.motor1_degree != motor1_base:
            if self.motor1_degree > motor1_base:
                self.motor1_degree -= 1
            else:
                self.motor1_degree += 1
            time.sleep(0.01)

        # 調整motor0
        while self.motor0_degree != motor0_base:
            if self.motor0_degree > motor0_base:
                self.motor0_degree -= 1
            else:
                self.motor0_degree += 1
            time.sleep(0.01)

        # 釋放方塊
        self.release_cube()

        # 重新定位手臂到前方
        self.motor0_degree = rospy.get_param('/Keyboard/motor0_base')
        self.motor1_degree = rospy.get_param('/Keyboard/motor1_base')

    # ======當鍵盤被放開時======
    def keyboard_release(self, key):
        try:
            if key.char in self.NORMAL_KEYBOARD_DICT.keys():
                self.direction = 'p'
                self.speed = 0
        except:
            if key == Key.up:
                self.dir_keyboard_state['up'] = False
            if key == Key.down:
                self.dir_keyboard_state['down'] = False
            if key == Key.left:
                self.dir_keyboard_state['left'] = False
            if key == Key.right:
                self.dir_keyboard_state['right'] = False

    # ======發布話題======
    def _send_keyboard(self):
        while not rospy.is_shutdown():
            self.control_arm()
            order = [self.direction, str(self.speed), str(self.motor0_degree), str(self.motor1_degree)]
            pub.publish(','.join(order))
            time.sleep(0.01)


if __name__ == '__main__':
    # 初始化
    rospy.init_node("keyboard_publisher", anonymous=True)
    pub = rospy.Publisher('keyboard_topic', String, queue_size=1)
    keyboard_manger = KeyboardManger()

    # 監聽鍵盤
    rospy.loginfo('Start listener keyboard...')
    rospy.loginfo('Press ESC exit.')
    with Listener(on_press=keyboard_manger.keyboard_press, on_release=keyboard_manger.keyboard_release) as listener:
        listener.join()
