#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Yolo_V4_Api import Detect
from Image_Callback import ImageCallback
import rospy
import cv2


def main():
    while not rospy.is_shutdown():
        img = img_callback.get_img()
        detections = yolo.detect_image(img)
        print(detections)

        cv2.imshow('', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    img_callback = ImageCallback()
    yolo = Detect()
    main()

