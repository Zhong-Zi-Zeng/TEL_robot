#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from Yolo_V4_Api import Detect
from simple_pid import PID
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

        # 初始化PID
        if rospy.get_param('/Pid/UsePid'):
            self.pid, self.sampleTime = self._init_pid()

        # 載入參數
        self.detect_limit = rospy.get_param('/DetectLimit')  # 檢測次數上限
        self.distance_threshold = rospy.get_param('/DistanceThreshold')  # 檢測最遠離閥值
        self.turn_degree = rospy.get_param('/TurnDegree')  # 找不到方框時調整機器人的角度
        self.hor_middle_point = rospy.get_param('/HorMiddlePoint')  # 水平中心點
        self.ver_middle_point = rospy.get_param('/VerMiddlePoint')  # 垂直中心點
        self.hor_error_range = rospy.get_param('/HorErrorRange')  # 在夾取方塊時允許的水平誤差範圍
        self.ver_error_range = rospy.get_param('/VerErrorRange')  # 在夾取方塊時允許的垂直誤差範圍
        self.grip_motor1_degree = rospy.get_param('GripMotor1Degree')  # 夾取物品時的motor1角度
        self.grip_motor2_degree = rospy.get_param('GripMotor2Degree')  # 夾取物品時的motor2角度
        self.release_motor1_degree = rospy.get_param('ReleaseMotor1Degree')  # 釋放物品時的motor2角度
        self.release_motor2_degree = rospy.get_param('ReleaseMotor2Degree')  # 釋放物品時的motor2角度

        # 設定變數
        self.now_direction = 'front'  # 紀錄當前機器人方向
        self.TEL_state = {'T': False, 'E': False, 'L': False}  # 紀錄TEL文字目前是否已經被夾取

    # =====初始化PID=====
    def _init_pid(self):
        kp = rospy.get_param('/Pid/Kp')
        ki = rospy.get_param('/Pid/Ki')
        kd = rospy.get_param('/Pid/Kd')
        target = rospy.get_param('/Pid/Target')
        sampleTime = rospy.get_param('/Pid/SampleTime')
        lowerLimit = rospy.get_param('/Pid/LowerLimit')
        upperLimit = rospy.get_param('/Pid/UpperLimit')

        return PID(kp, ki, kd, setpoint=target, output_limits=(lowerLimit, upperLimit)), sampleTime

    def start(self):
        # 尋找有無TEL
        detect_temp = self._find_TEL()

        # 偵測達到上限後若暫存區依舊為空則先將機器人往左擺動後重新判斷
        # 還是為空再向右擺動判斷，還是為空則將機器人定位到方框後
        if not detect_temp:
            self._correction_robot()
            self.start()
            return

        # 過濾掉太遠的物體
        detect_temp = self._filter_detect_temp(detect_temp)

        # 按照順序進行定位
        self._localization_robot(detect_temp)

    # =====尋找有無TEL=====
    def _find_TEL(self):
        detect_temp = {}

        for i in range(self.detect_limit):
            img = self.img_queue.get_img()
            detections = self.network.detect_image(img)

            for label, _, bbox in detections:
                x, y, w, h = bbox
                # 如果有重複的話只存放離機身較近的那點
                if label in detect_temp.keys():
                    detect_temp[label] = [x, y, w, h] if y > detect_temp[label][1] else detect_temp[label]
                else:
                    detect_temp[label] = [x, y, w, h]

        return detect_temp

    # =====判斷物體距離是否大於閥值，若大於閥值則去除掉=====
    def _filter_detect_temp(self, detect_temp):
        for label in detect_temp.keys():
            distance = self._get_distance(detect_temp[label])
            if distance > self.distance_threshold:
                del detect_temp[label]

        return detect_temp

    # =====校正機器人角度=====
    def _correction_robot(self):
        # 向右轉
        if self.now_direction == 'front':
            self.uart_api.send_order(degree=self.turn_degree)
            self.now_direction = 'right'
        # 向左轉
        elif self.now_direction == 'right':
            self.uart_api.send_order(degree=-self.turn_degree * 2)
            self.now_direction = 'left'
        # 定位到方框後
        else:
            self.uart_api.send_special_order(action='f')
            self.now_direction = 'front'
            return

        self.uart_api.send_special_order(action='z')

    # =====定位機器人=====
    def _localization_robot(self, detect_temp):

        for char in detect_temp.keys:
            while True:
                now_detect = self._find_TEL()

                try:
                    x, y, w, h = now_detect[char]
                    distance = self._get_distance(now_detect[char])
                    c_x = x + w / 2

                    # 判斷左右是否需要調整
                    if not self.hor_middle_point - self.hor_error_range <= c_x <= self.hor_middle_point + self.hor_error_range:
                        if c_x > self.hor_middle_point:
                            self.uart_api.send_order(direction='d', value=str(abs(self.hor_middle_point - c_x)))
                        else:
                            self.uart_api.send_order(direction='a', value=str(abs(self.hor_middle_point - c_x)))
                    # 判斷前後是否需要調整
                    elif not self.ver_middle_point - self.ver_error_range <= distance <= self.ver_middle_point + self.ver_error_range:
                        if distance > self.ver_middle_point:
                            self.uart_api.send_order(direction='s', value=str(abs(self.hor_middle_point - distance)))
                        else:
                            self.uart_api.send_order(direction='w', value=str(abs(self.hor_middle_point - distance)))
                    # 夾取方塊
                    else:
                        successfully = self._grip_cube()
                        if not successfully:
                            continue
                        else:
                            break
                except:
                    pass

    # =====夾取方塊=====
    def _grip_cube(self):
        # 將爪子定位到吸取位置
        self.uart_api.send_order(motor_1=str(self.grip_motor1_degree), motor_2=str(self.grip_motor2_degree))
        time.sleep(0.5)

        # 吸取方塊
        successfully = self.uart_api.send_special_order(action='a')

        # 沒有成功吸取則重新定位
        if not successfully:
            return False

        # 將爪子定位到放料位置
        self.uart_api.send_order(motor_1=str(self.release_motor1_degree), motor_2=str(self.release_motor2_degree))
        time.sleep(0.5)

        # 釋放方塊
        self.uart_api.send_special_order(action='b')

    # =====取得與方塊間的距離=====
    def _get_distance(self, bbox):
        """
        :param bbox: 偵測方框
        :return distance: 單位(m)
        """
        x, y, w, h = bbox
        c_x = x + w // 2
        c_y = y + h // 2

        depth_img = self.img_queue.get_depth_img()
        distance = depth_img[c_x, c_y]

        return distance
