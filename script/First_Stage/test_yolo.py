#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Yolo_V4_Api import Detect
from Image_Callback import ImageCallback
import rospy
import cv2

color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
label = {'T': 0, 'E': 1, 'L': 2}


# =====取得與方塊間的距離=====
def _get_distance(c_x, c_y):
    depth_img = img_callback.get_depth_img()
    distance = depth_img[c_y, c_x]

    return distance


def main():
    while not rospy.is_shutdown():
        img = img_callback.get_img()
        detections = yolo.detect_image(img)
        if len(detections):
            for detection in detections:
                classId, confs, box = detection
                box = list(map(int, box))
                distance = _get_distance(box[0], box[1])
                x = box[0] - (box[2] - 1) // 2
                y = box[1] - (box[3] - 1) // 2

                cv2.rectangle(img, (x, y), (x + box[2], y + box[3]),
                              color=color[label[classId]], thickness=2)
                text = '%s: ,score:%.2s , d:%.2f' % (classId, confs, distance)
                cv2.putText(img, text, (box[0], box[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            color=color[label[classId]], thickness=2)

        cv2.imshow('', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    img_callback = ImageCallback()
    yolo = Detect()
    main()
