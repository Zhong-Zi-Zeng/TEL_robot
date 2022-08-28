#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from Yolo_V4_Api import Detect

class Level1:
    def __init__(self):
        # 初始化Yolo檢測
        self.network = Detect()

    def start(self, img):
        # detections = network.detect_image(img)
        # print(detections)
        pass