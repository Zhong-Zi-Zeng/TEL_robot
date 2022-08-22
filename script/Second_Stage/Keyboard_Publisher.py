#!/usr/bin/env python
import pynput.keyboard
import rospy
from pynput.keyboard import Key
from pynput.keyboard import Listener
from std_msgs.msg import String

# ======鍵盤對應的編碼======
KEYBOARD_DICT = {
    Key.up: 'up',
    Key.down: 'down',
    Key.left: 'left',
    Key.right: 'right',
}

# ======當鍵盤被按下時======
def keyboard_press(key):
    try:
        # 按下一般按鍵
        if key in KEYBOARD_DICT.keys():
            send_keyboard(KEYBOARD_DICT[key.char])
        # 按下esc鍵
        elif key == Key.esc:
            rospy.loginfo('Press Esc')
            return False
    # 按下一般特殊按鍵
    except AttributeError:
        if key in KEYBOARD_DICT.keys():
            send_keyboard(KEYBOARD_DICT[key])

# ======發布話題======
def send_keyboard(order):
    pub.publish(order)

if __name__ == '__main__':
    rospy.init_node("imu_publisher")
    pub = rospy.Publisher('keyboard_topic', String, queue_size=1)

    # 監聽鍵盤
    rospy.loginfo('Start listener keyboard...')
    with Listener(on_press=keyboard_press) as listener:
        listener.join()

