#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np

from Yolo_V4_Api import Detect
from Uart_Api import UartApi
from Debug import Debug
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

        # 初始話Debug
        self.debug = Debug()

        # 載入參數
        self.velocity_baseline = rospy.get_param('/VelocityBaseline')  # 最低速度基準線
        self.check_distance_threshold = rospy.get_param('/CheckDistanceThreshold')  # 檢測最遠離閥值
        self.pan_speed = rospy.get_param('/PanSpeed')  # 找不到方框時調整機器人平移的速度
        self.pan_time = rospy.get_param('/PanTime')  # 平移時間
        self.side_hor_middle_point = rospy.get_param('/SideHorMiddlePoint')  # 側邊水平中心點
        self.side_ver_middle_point = rospy.get_param('/SideVerMiddlePoint')  # 側邊垂直中心點
        self.top_hor_middle_point = rospy.get_param('/TopHorMiddlePoint')  # 上方水平中心點
        self.top_ver_middle_point = rospy.get_param('/TopVerMiddlePoint')  # 上方垂直中心點
        self.hor_error_range = rospy.get_param('/HorErrorRange')  # 在夾取方塊時允許的水平誤差範圍
        self.ver_error_range = rospy.get_param('/VerErrorRange')  # 在夾取方塊時允許的垂直誤差範圍

        self.init_motor1_degree = rospy.get_param('/InitMotor1Degree')  # motor1初始角度
        self.init_motor2_degree = rospy.get_param('/InitMotor2Degree')  # motor2初始角度
        self.check_motor1_degree = rospy.get_param('/CheckMotor1Degree')  # 檢查是否有夾到物品時的motor1角度
        self.check_motor2_degree = rospy.get_param('/CheckMotor2Degree')  # 檢查是否有夾到品時的motor2角度
        self.grip_motor1_degree = rospy.get_param('/GripMotor1Degree')  # 夾取物品時的motor1角度
        self.grip_motor2_degree = rospy.get_param('/GripMotor2Degree')  # 夾取物品時的motor2角度
        self.release_motor1_degree = rospy.get_param('/ReleaseMotor1Degree')  # 釋放物品時的motor2角度
        self.release_motor2_degree = rospy.get_param('/ReleaseMotor2Degree')  # 釋放物品時的motor2角度

        # 設定變數
        self.now_direction = "initial"  # 紀錄當前機器人方向
        self.now_degree = 0  # 紀錄機器人當前角度
        self.now_state = "collect"  # 紀錄目前狀態，一開始為蒐集方塊
        self.motor1_now_degree = self.init_motor1_degree  # motor1目前角度
        self.motor2_now_degree = self.init_motor2_degree  # motor2目前角度
        self.banned_list = []  # 用來存放暫時夾不到的方塊
        self.TEL_state = {'T': False, 'E': False, 'L': False}  # 紀錄TEL文字目前是否已經被夾取

    def start(self):
        self.debug.debug_info('Level1 Start!')
        self.uart_api.send_special_order(action='n')  # 更新初始角度
        self.uart_api.send_special_order(action='t')  # 開啟回正
        while True:
            # 尋找有無TEL
            detect_temp = self._find_TEL()

            # 過濾掉存在禁止列表中的方塊、已經夾到的方塊
            detect_temp = self._filter_detect_temp(detect_temp)

            # 偵測達到上限後若暫存區依舊為空則先將機器人往左擺動後重新判斷
            # 還是為空再向右擺動判斷，還是為空則將機器人定位到方框後
            if len(detect_temp) == 0 and self.now_direction != 'back left':
                self._correction_robot()
                continue

            # 找離目前最近的方塊並定位
            nearer_cube = self._find_nearer_cube(detect_temp)
            result = self._localization_robot(nearer_cube)

            # 夾取方塊
            if result and self._grip_cube():
                # 將爪子定位到放料位置
                self._control_motor2(target_angel=self.release_motor2_degree)
                self._control_motor1(target_angel=self.release_motor1_degree)

                # 釋放方塊
                self.debug.debug_info('Put down cube')
                self.uart_api.send_special_order(action='b')
                time.sleep(0.5)

                # 重新定位手臂到初始位置
                self.debug.debug_info('Reset arm')
                self._control_motor1(target_angel=self.init_motor1_degree)
                self._control_motor2(target_angel=self.init_motor2_degree)

                self.TEL_state[list(nearer_cube.keys())[0]] = True

            # 若已經定位到方框後但還是找不到方塊則直接去到料
            if self.now_direction == 'back left':
                for key in self.TEL_state.keys():
                    self.TEL_state[key] = True

            # 判斷是否還有文字方塊未被夾取
            if False in self.TEL_state.values():
                continue

            self.debug.debug_info('Go to target point!')
            # 定位到終點方框
            if 'back' in self.now_direction:
                self.uart_api.send_special_order(action='h')
            elif 'side' in self.now_direction:
                self.uart_api.send_special_order(action='v')
            else:
                self.uart_api.send_special_order(action='g')

            # 疊方塊
            self.now_degree = 0
            self._stack_cube()

            # 前往第二關
            self.debug.debug_info('Go to Level 2!')
            self.uart_api.send_special_order(action='w')

            return True

    # =====尋找有無TEL=====
    def _find_TEL(self):
        self.debug.debug_info('Find TEL....')

        detect_temp = {}
        img = self.img_queue.get_img()
        detections = self.network.detect_image(img)

        for label, _, bbox in detections:
            c_x, c_y, w, h = list(map(int, bbox))
            # 如果有重複的話只存放離機身較近的那點
            if label in detect_temp.keys():
                detect_temp[label] = [c_x, c_y, w, h] if c_y > detect_temp[label][1] else detect_temp[label]
            else:
                detect_temp[label] = [c_x, c_y, w, h]

        self.debug.debug_info('Find result:', detect_temp.keys())

        return detect_temp

    # =====已夾取、存在禁止列表中的方塊都要刪掉====
    def _filter_detect_temp(self, detect_temp):
        copy_detect_temp = detect_temp.copy()
        self.debug.debug_info('Filter out cube')

        for key in detect_temp.keys():
            if self.TEL_state[key] or key in self.banned_list:
                del copy_detect_temp[key]

        self.debug.debug_info('Now detect temp:', copy_detect_temp)

        return copy_detect_temp

    # =====校正機器人=====
    def _correction_robot(self):
        self.debug.debug_info('Can not find TEL, modify robot direction.')

        # 從起點定位到方框前
        if self.now_direction == "initial":
            self.banned_list = []  # 清空禁止列表
            self.debug.debug_info('Now direction is front.')
            self.uart_api.send_special_order(action='e')
            self.now_direction = 'front'
            self.now_degree = 0
            return

        # 從前方定位到方框側邊
        elif self.now_direction == 'front left':
            self.banned_list = []  # 清空禁止列表
            self.debug.debug_info('Now direction is side.')
            self.uart_api.send_special_order(action='u')
            self.now_direction = 'side'
            self.now_degree = 270
            return

        # 從側邊定位到方框後面
        elif self.now_direction == 'side left':
            self.banned_list = []  # 清空禁止列表
            self.debug.debug_info('Now direction is back.')
            self.uart_api.send_special_order(action='f')
            self.now_direction = 'back'
            self.now_degree = 180
            return

        # 由前向右平移
        elif self.now_direction == 'front':
            self.uart_api.send_special_order(action='o')  # 先將機器人定位到方框前方
            self.uart_api.send_order(direction='d', value=str(self.pan_speed), degree=str(0))
            self.now_direction = 'front right'

        # 由側邊向右平移
        elif self.now_direction == 'side':
            self.uart_api.send_special_order(action='p')  # 先將機器人定位到方框側邊
            self.uart_api.send_order(direction='d', value=str(self.pan_speed), degree=str(270))
            self.now_direction = 'side right'

        # 由後向右平移
        elif self.now_direction == 'back':
            self.uart_api.send_special_order(action='q')  # 先將機器人定位到方框後方
            self.uart_api.send_order(direction='d', value=str(self.pan_speed), degree=str(180))
            self.now_direction = 'back right'

        # 由前向左平移
        elif self.now_direction == 'front right':
            self.uart_api.send_special_order(action='o')  # 先將機器人定位到方框前方
            self.uart_api.send_order(direction='a', value=str(self.pan_speed), degree=str(0))
            self.now_direction = 'front left'

        # 由側邊向左平移
        elif self.now_direction == 'side right':
            self.uart_api.send_special_order(action='p')  # 先將機器人定位到方框側邊
            self.uart_api.send_order(direction='a', value=str(self.pan_speed), degree=str(270))
            self.now_direction = 'side left'

        # 由後向左平移
        elif self.now_direction == 'back right':
            self.uart_api.send_special_order(action='q')  # 先將機器人定位到方框後方
            self.uart_api.send_order(direction='a', value=str(self.pan_speed), degree=str(180))
            self.now_direction = 'back left'

        self.debug.debug_info('Now direction is', self.now_direction)
        time.sleep(self.pan_time)  # 平移時間
        self.uart_api.send_order(direction='p', degree=str(self.now_degree))

        # self.uart_api.send_special_order(action='z')

    def reset(self):
        if ("front" in self.now_direction):
            self.uart_api.send_special_order(action='o')  # 先將機器人定位到方框前方
        elif ("side" in self.now_direction):
            self.uart_api.send_special_order(action='p')  # 先將機器人定位到方框側邊
        else:
            self.uart_api.send_special_order(action='q')  # 先將機器人定位到方框後方

    # =====找出離機器人最近的方塊=====
    @staticmethod
    def _find_nearer_cube(detect_temp):
        if len(detect_temp) == 0:
            return

        maximize_c_y = -np.Inf
        result = None

        for char, bbox in detect_temp.items():
            c_x, c_y, w, h = bbox
            if c_y > maximize_c_y:
                maximize_c_y = c_y
                result = {char: [c_x, c_y, w, h]}

        return result

    # =====定位機器人夾取方塊=====
    def _localization_robot(self, nearer_cube):
        if nearer_cube is None:
            return

        char = list(nearer_cube.keys())[0]

        # 判斷看到的點是在上方還是側邊給與不同的定位點
        hor_middle_point, ver_middle_point = self._check_point(nearer_cube[char][0], nearer_cube[char][1])

        while True:
            self.debug.debug_info('Localization', char, 'cube')

            now_detect = self._find_TEL()
            try:
                c_x, c_y, w, h = now_detect[char]

                # 判斷左右是否需要調整
                if not hor_middle_point - self.hor_error_range <= c_x <= hor_middle_point + self.hor_error_range:
                    delay_time = abs(c_x - hor_middle_point) / 150 + 0.1
                    if c_x > hor_middle_point:
                        self.uart_api.send_order(direction='d', value=str(self.velocity_baseline + 5),
                                                 degree=str(self.now_degree), motor_1=str(self.motor1_now_degree),
                                                 motor_2=str(self.motor2_now_degree))
                        self.debug.debug_info('Move Right')
                    else:
                        self.uart_api.send_order(direction='a', value=str(self.velocity_baseline + 5),
                                                 degree=str(self.now_degree), motor_1=str(self.motor1_now_degree),
                                                 motor_2=str(self.motor2_now_degree))
                        self.debug.debug_info('Move Left')

                    time.sleep(delay_time)
                    self.uart_api.send_order(direction='p', degree=str(self.now_degree),
                                             motor_1=str(self.motor1_now_degree),
                                             motor_2=str(self.motor2_now_degree))
                    hor_correct = False
                else:
                    hor_correct = True

                # 判斷前後是否需要調整
                if not ver_middle_point - self.ver_error_range <= c_y <= ver_middle_point + self.ver_error_range:
                    delay_time = abs(c_y - ver_middle_point) / 150 + 0.1
                    if c_y > ver_middle_point:
                        self.uart_api.send_order(direction='s', value=str(self.velocity_baseline),
                                                 degree=str(self.now_degree), motor_1=str(self.motor1_now_degree),
                                                 motor_2=str(self.motor2_now_degree))
                        self.debug.debug_info('Move Back')
                    else:
                        self.uart_api.send_order(direction='w', value=str(self.velocity_baseline),
                                                 degree=str(self.now_degree), motor_1=str(self.motor1_now_degree),
                                                 motor_2=str(self.motor2_now_degree))
                        self.debug.debug_info('Move Front')

                    time.sleep(delay_time)
                    self.uart_api.send_order(direction='p', degree=str(self.now_degree),
                                             motor_1=str(self.motor1_now_degree),
                                             motor_2=str(self.motor2_now_degree))
                    ver_correct = False
                else:
                    ver_correct = True

                # 判斷機器人有沒有撞到方框，若撞到則將此方塊加入到禁止列表中並退出迴圈
                if self.uart_api.send_special_order(action='m') == False and self.now_state == "collect":
                    self.banned_list.append(char)
                    self.debug.debug_info('Hit the box and the letter', char, 'is added to the banned list')
                    self.reset()
                    break

                # 判斷機器人會不會掉下去，如果會掉下去則放棄此方塊並退出迴圈
                if self.uart_api.send_special_order(action='r') == False and self.now_state == "collect":
                    self.TEL_state[char] = True
                    self.debug.debug_info('Falling warring give up the letter', char)
                    self.reset()
                    break

                # 回傳定位完成
                if hor_correct and ver_correct:
                    self.debug.debug_info('Positioning completed start grip', char)
                    return True
            except:
                self.debug.debug_info('Something Error')
                return False

    # =====疊起方塊=====
    def _stack_cube(self):
        self.TEL_state = {'T': False, 'E': False, 'L': False}  # 紀錄目前TEL是否都已疊起

        # 定位完成後吸取方塊->檢查是否有夾到->將手臂移動到右方
        while True:
            self.debug.debug_info("Position First Cube")
            # 對離機器人最近的方塊進行定位並記錄為第一個方塊
            detect_temp = self._find_TEL()
            first_cube = self._find_nearer_cube(detect_temp)
            try:
                first_cube_char = list(first_cube.keys())[0]
                result = self._localization_robot(first_cube)
                if result:
                    if self._grip_cube():
                        # 吸著方塊
                        self.TEL_state[first_cube_char] = True
                        self._control_motor2(target_angel=78)
                        self._control_motor1(target_angel=90)
                        break
                    else:
                        self.TEL_state[first_cube_char] = False
                        continue
            except:
                self.debug.debug_info("Can't Find First Cube")

        # 尋找下一個方塊並進行定位->將方塊放置在上方
        self.now_state = "put_cube_1"
        while True:
            self.debug.debug_info("Position Second Cube")
            detect_temp = self._find_TEL()
            next_cube = self._find_nearer_cube(detect_temp)
            try:

                next_cube_char = list(first_cube.keys())[0]
                result = self._localization_robot(next_cube)

                if result:
                    # 放下第一個方塊
                    self.debug.debug_info('Put down cube')
                    self._control_motor1(target_angel=115)
                    self._control_motor2(target_angel=34)
                    time.sleep(2)
                    self.TEL_state[next_cube_char] = True
                    self.uart_api.send_special_order(action='b')
                    self._control_motor1(target_angel=self.init_motor1_degree)
                    self._control_motor2(target_angel=self.init_motor2_degree)
                    break
            except:
                self.debug.debug_info("Can't Find Second Cube")

    # ===== 判斷偵測到的方塊的正上方還是側邊，設定不同的中心點=====
    def _check_point(self, c_x, c_y):
        if self.now_state == "collect":
            # 取得偵測點的距離
            dis = self._get_distance(c_x, c_y)

            # 取得偵測點上方的距離
            up_dis = self._get_distance(c_x, c_y - 20)

            # 判斷偵測到的方塊的正上方還是側邊
            if up_dis > dis:
                self.debug.debug_info("Top point")
                return self.top_hor_middle_point, self.top_ver_middle_point
            else:
                self.debug.debug_info("Side point")
                return self.side_hor_middle_point, self.side_ver_middle_point
        elif self.now_state == "put_cube_1":
            return 500, 405

    # =====夾取方塊
    def _grip_cube(self):
        # 先停止機器人
        self.uart_api.send_order(direction='p', degree=str(self.now_degree), motor_1=str(self.motor1_now_degree),
                                 motor_2=str(self.motor2_now_degree))

        # 將爪子定位到吸取位置
        self.debug.debug_info('Positioning arm...')
        self._control_motor1(target_angel=self.grip_motor1_degree)
        self._control_motor2(target_angel=self.grip_motor2_degree)
        time.sleep(0.01)

        # 吸取方塊
        self.debug.debug_info('Gripping cube...')
        self.uart_api.send_special_order(action='a')
        time.sleep(0.5)

        # 確認是否有吸到方塊，沒有成功吸取則重新定位手臂並把幫補關掉
        self.debug.debug_info('Check cube...')
        successfully = self._check_grip_cube()
        if not successfully:
            self.debug.debug_info('Grip failed..')
            self.motor1_now_degree = self.init_motor1_degree
            self.motor2_now_degree = self.init_motor2_degree
            self.uart_api.send_order(direction='p', degree=str(self.now_degree))
            time.sleep(0.2)
            self.uart_api.send_special_order(action='b')
            return False

        return True

    # =====確認是否夾到方塊=====
    def _check_grip_cube(self):
        self._control_motor2(target_angel=self.check_motor2_degree)
        self._control_motor1(target_angel=self.check_motor1_degree)

        time.sleep(1)
        distance = self._get_distance(320, 122)
        self.debug.debug_info("Check distance:", distance)

        if distance > self.check_distance_threshold:
            return False

        return True

    # =====取得與方塊間的距離=====
    def _get_distance(self, c_x, c_y):
        depth_img = self.img_queue.get_depth_img()
        distance = depth_img[c_y, c_x]

        if distance == 0:
            return 100
        else:
            return distance

    # =====操控motor1=====
    def _control_motor1(self, target_angel):
        while self.motor1_now_degree != target_angel:
            self.uart_api.send_order(motor_1=str(self.motor1_now_degree), motor_2=str(self.motor2_now_degree),
                                     degree=str(self.now_degree))

            if self.motor1_now_degree < target_angel:
                self.motor1_now_degree += 1
            else:
                self.motor1_now_degree -= 1
            time.sleep(0.01)

    # =====操控motor2=====
    def _control_motor2(self, target_angel):
        while self.motor2_now_degree != target_angel:
            self.uart_api.send_order(motor_1=str(self.motor1_now_degree), motor_2=str(self.motor2_now_degree),
                                     degree=str(self.now_degree))

            if self.motor2_now_degree < target_angel:
                self.motor2_now_degree += 1
            else:
                self.motor2_now_degree -= 1
            time.sleep(0.01)
