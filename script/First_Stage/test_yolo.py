#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Yolo_V4_Api import Detect
from Image_Callback import ImageCallback
import rospy
import cv2

color = [(255,0,0),(0,255,0),(0,0,255)]
label = ['T', 'E', 'L']

def main():
    while not rospy.is_shutdown():
        img = img_callback.get_img()
        classes, confs, boxes = yolo.detect_image(img)

        for (classId, score, box) in zip(classes, confs, boxes):
            cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                          color=color[classId], thickness=2)
            text = '%s: %.2f' % (label[classId], score)
            cv2.putText(img, text, (box[0], box[1] + 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color=color[classId], thickness=2)

        cv2.imshow('', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    img_callback = ImageCallback()
    yolo = Detect()
    main()

