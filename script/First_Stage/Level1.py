#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from Yolo_V4_Api import Detect
from Uart_Api import UartApi
import rospy
import time


class Level1:
    def __init__(self, img_queue):
        # 影像監聽
        self.img_queue = img_queue

        # 初始化Yolo檢測
        self.network = Detect()

        # 初始化Uart Api
        self.uart_api = UartApi()

        # 載入參數
        self.debug = rospy.get_param('/Debug')  # Debug模式
        self.detect_limit = rospy.get_param('/DetectLimit')  # 檢測次數上限
        self.velocity_baseline = rospy.get_param('/VelocityBaseline')  # 最低速度基準線
        self.distance_threshold = rospy.get_param('/DistanceThreshold')  # 檢測最遠離閥值
        self.check_distance_threshold = rospy.get_param('/CheckDistanceThreshold')  # 檢測最遠離閥值
        self.turn_degree = rospy.get_param('/TurnDegree')  # 找不到方框時調整機器人的角度
        self.hor_middle_point = rospy.get_param('/HorMiddlePoint')  # 水平中心點
        self.ver_middle_point = rospy.get_param('/VerMiddlePoint')  # 垂直中心點
        self.hor_error_range = rospy.get_param('/HorErrorRange')  # 在夾取方塊時允許的水平誤差範圍
        self.ver_error_range = rospy.get_param('/VerErrorRange')  # 在夾取方塊時允許的垂直誤差範圍
        self.check_motor1_degree = rospy.get_param('/CheckMotor1Degree')  # 檢查是否有夾到物品時的motor1角度
        self.check_motor2_degree = rospy.get_param('/CheckMotor2Degree')  # 檢查是否有夾到品時的motor2角度
        self.grip_motor1_degree = rospy.get_param('/GripMotor1Degree')  # 夾取物品時的motor1角度
        self.grip_motor2_degree = rospy.get_param('/GripMotor2Degree')  # 夾取物品時的motor2角度
        self.release_motor1_degree = rospy.get_param('/ReleaseMotor1Degree')  # 釋放物品時的motor2角度
        self.release_motor2_degree = rospy.get_param('/ReleaseMotor2Degree')  # 釋放物品時的motor2角度

        # 設定變數
        self.now_direction = None  # 紀錄當前機器人方向
        self.TEL_state = {'T': False, 'E': False, 'L': False}  # 紀錄TEL文字目前是否已經被夾取

    def start(self):
        # 尋找有無TEL
        detect_temp = self._find_TEL()

        # 過濾掉太遠的方塊、已經夾到的方塊
        detect_temp = self._filter_detect_temp(detect_temp)

        # 偵測達到上限後若暫存區依舊為空則先將機器人往左擺動後重新判斷
        # 還是為空再向右擺動判斷，還是為空則將機器人定位到方框後
        if not detect_temp and self.now_direction != 'bake left':
            self._correction_robot()
            self.start()

        # 按照順序進行定位
        self._localization_robot(detect_temp)

        # 若已經定位到方框後但還是找不到方塊則直接去到料
        if self.now_direction == 'back left':
            for key in self.TEL_state.keys():
                self.TEL_state[key] = True

        # 判斷是否還有文字方塊未被夾取
        if False in self.TEL_state.values():
            self.start()

        # 定位到終點方框
        if 'back' in self.now_direction:
            self.uart_api.send_special_order(action='h')
        else:
            self.uart_api.send_special_order(action='g')

        # 到料
        self.uart_api.send_special_order(action='c')
        time.sleep(3)
        self.uart_api.send_special_order(action='d')

        # 前往第二關
        self.uart_api.send_special_order(action='j')

        if self.debug:
            print('Go to Level 2!')

        return True

    # =====尋找有無TEL=====
    def _find_TEL(self):
        detect_temp = {}

        if self.debug:
            print('Find TEL....')

        img = self.img_queue.get_img()
        detections = self.network.detect_image(img)

        for label, _, bbox in detections:
            c_x, c_y, w, h = list(map(int, bbox))

            # 如果有重複的話只存放離機身較近的那點
            if label in detect_temp.keys():
                detect_temp[label] = [c_x, c_y, w, h] if c_y > detect_temp[label][1] else detect_temp[label]
            else:
                detect_temp[label] = [c_x, c_y, w, h]

        if self.debug:
            print('Find result:', detect_temp.keys())

        return detect_temp

    # =====判斷物體距離是否大於閥值，若大於閥值則去除掉=====
    def _filter_detect_temp(self, detect_temp):
        copy_detect_temp = detect_temp.copy()

        if self.debug:
            print('Original detect temp:', detect_temp)
            print('Filter out the too-far cube.')

        for key in detect_temp.keys():
            distance = self._get_distance(c_x=detect_temp[key][0], c_y=detect_temp[key][1])
            if distance > self.distance_threshold or self.TEL_state[key]:
                del copy_detect_temp[key]

        if self.debug:
            print('Now detect temp:', copy_detect_temp)

        return copy_detect_temp

    # =====校正機器人=====
    def _correction_robot(self):
        if self.debug:
            print('Can not find TEL, modify robot direction.')

        # 一開始定位到方框前
        if self.now_direction is None:
            self.uart_api.send_special_order(action='e')
            self.now_direction = 'front'

        # 由前向右轉
        elif self.now_direction == 'front':
            self.uart_api.send_order(degree=str(self.turn_degree))
            self.now_direction = 'right'

        # 由後向右轉
        elif self.now_direction == 'back':
            self.uart_api.send_order(degree=str(180 + self.turn_degree))
            self.now_direction = 'back right'

        # 由前向左轉
        elif self.now_direction == 'right':
            self.uart_api.send_order(degree=str(360 - self.turn_degree))
            self.now_direction = 'left'

        # 由後向左轉
        elif self.now_direction == 'bake right':
            self.uart_api.send_order(degree=str(180 - self.turn_degree))
            self.now_direction = 'back left'

        # 定位到方框後
        elif self.now_direction != 'bake left':
            self.uart_api.send_special_order(action='f')
            self.now_direction = 'back'
            return

        self.uart_api.send_special_order(action='z')

    # =====定位機器人夾取方塊=====
    def _localization_robot(self, detect_temp):
        for char in detect_temp.keys():
            successfully = False

            while not successfully:
                now_detect = self._find_TEL()
                if self.debug:
                    print('Gripping {} cube'.format(char))

                try:
                    c_x, c_y, w, h = now_detect[char]

                    # 判斷左右是否需要調整
                    if not self.hor_middle_point - self.hor_error_range <= c_x <= self.hor_middle_point + self.hor_error_range:
                        if c_x > self.hor_middle_point:
                            self.uart_api.send_order(direction='d', value=str(
                                abs(self.hor_middle_point - c_x) / 10 + self.velocity_baseline))
                            if self.debug:
                                print('Move Right')
                        else:
                            self.uart_api.send_order(direction='a', value=str(
                                abs(self.hor_middle_point - c_x) / 10 + self.velocity_baseline))
                            if self.debug:
                                print('Move Left')
                    # 判斷前後是否需要調整
                    elif not self.ver_middle_point - self.ver_error_range <= c_y <= self.ver_middle_point + self.ver_error_range:
                        if c_y > self.ver_middle_point:
                            self.uart_api.send_order(direction='s', value=str(
                                abs(self.hor_middle_point - c_y) / 10 + self.velocity_baseline))
                            if self.debug:
                                print('Move Back')
                        else:
                            self.uart_api.send_order(direction='w', value=str(
                                abs(self.hor_middle_point - c_y) / 10 + self.velocity_baseline))
                            if self.debug:
                                print('Move Front')
                    # 夾取方塊，若夾取到則把此字母狀態改為True，沒夾到則直接換夾下一個
                    else:
                        if self.debug:
                            print('Positioning completed start grip {}'.format(char))

                        if self._grip_cube():
                            self.TEL_state[char] = True
                        else:
                            break
                except:
                    pass

    # =====夾取方塊=====
    def _grip_cube(self):
        # 先停止機器人
        self.uart_api.send_order(direction='p')

        # 將爪子定位到吸取位置
        self.uart_api.send_order(motor_1=str(self.grip_motor1_degree), motor_2=str(self.grip_motor2_degree))
        if self.debug:
            print('Positioning arm...')

        # 吸取方塊
        if self.debug:
            print('Gripping cube...')
        self.uart_api.send_special_order(action='a')
        time.sleep(0.5)

        # 確認是否有吸到方塊，沒有成功吸取則重新定位手臂
        if self.debug:
            print('Check cube...')
        successfully = self._check_grip_cube()
        if not successfully:
            self.uart_api.send_order()
            return False

        # 將爪子定位到放料位置
        motor1_degree = self.check_motor1_degree
        motor2_degree = self.check_motor2_degree
        while motor2_degree != self.release_motor2_degree:
            self.uart_api.send_order(motor_2=str(motor2_degree))
            motor2_degree += 1
            time.sleep(0.01)

        while motor1_degree != self.release_motor1_degree:
            self.uart_api.send_order(motor_1=str(motor1_degree))
            motor1_degree -= 1
            time.sleep(0.01)

        # 釋放方塊
        if self.debug:
            print('Put down cube')
        self.uart_api.send_special_order(action='b')

        return True

    # =====確認是否夾到方塊方塊=====
    def _check_grip_cube(self):
        # 將爪子定位到偵測位置
        motor1_degree = self.grip_motor1_degree
        motor2_degree = self.grip_motor2_degree

        while motor2_degree != self.check_motor2_degree:
            self.uart_api.send_order(motor_2=str(motor2_degree))
            motor2_degree -= 1
            time.sleep(0.01)

        while motor1_degree != self.check_motor1_degree:
            self.uart_api.send_order(motor_1=str(motor1_degree))
            motor1_degree += 1
            time.sleep(0.01)

        distance = self._get_distance(270, 43)
        if distance > self.check_distance_threshold:
            return False

        return True

    # =====取得與方塊間的距離=====
    def _get_distance(self, c_x, c_y):
        depth_img = self.img_queue.get_depth_img()
        distance = depth_img[c_y, c_x]

        return distance
