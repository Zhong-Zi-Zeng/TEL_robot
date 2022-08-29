#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from Yolo_V4_Api import Detect
from simple_pid import PID
from Uart_Api import UartApi
import rospy

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

        # 檢測次數上限
        self.detect_limit = rospy.get_param('/DetectLimit')

        # 檢測暫存
        self.detect_temp = {}

        # 檢測最遠離閥值
        self.distance_threshold = rospy.get_param('/DistanceThreshold')

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
        self.find_TEL()

        # 偵測達到上限後若暫存區依舊為空則移動機器人到起點方框後
        if not self.detect_temp:
            self.uart_api.send_special_order(action='f')

        # 判斷物體距離是否大於閥值，若大於閥值則去除掉
        for label in self.detect_temp.keys():
            distance = self.get_distance(self.detect_temp[label])
            if distance > self.distance_threshold:
                del self.detect_temp[label]


    def find_TEL(self):
        for i in range(self.detect_limit):
            img = self.img_queue.get_img()
            detections = self.network.detect_image(img)

            for label, _, bbox in detections:
                x, y, w, h = bbox

                # 如果有重複的話只存放離機身較近的那點
                if label in self.detect_temp.keys():
                    self.detect_temp[label] = [x, y, w, h] if y > self.detect_temp[label][1] else self.detect_temp[
                        label]
                else:
                    self.detect_temp[label] = [x, y, w, h]



    def get_distance(self, bbox):
        """
        :param bbox: 偵測方框
        :return distance: 單位(m)
        """
        x, y, w, h = bbox
        distance = 0

        return distance
