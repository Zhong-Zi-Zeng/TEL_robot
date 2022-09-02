#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from Yolo_V4_Api import Detect
from Image_Callback import ImageCallback
import rospy
import cv2

color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
label = {'T':0, 'E':1, 'L':2}


def main():
    while not rospy.is_shutdown():
        img = img_callback.get_img()
        detections = yolo.detect_image(img)
        if len(detections):
            for detection in detections:
                classId, confs, box = detection
                box = list(map(int, box))

                cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                              color=color[label[classId]], thickness=2)
                text = '%s: %.2s' % (classId, confs)
                cv2.putText(img, text, (box[0], box[1] + 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            color=color[label[classId]], thickness=2)

        cv2.imshow('', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    img_callback = ImageCallback()
    yolo = Detect()
    main()
