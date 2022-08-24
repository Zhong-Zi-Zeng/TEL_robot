import cv2
import sys
from Yolo_V4_Api import Detect


network = Detect()

cap = cv2.VideoCapture(0)


while True:
    ret, img = cap.read()

    detect = network.detect_image(img)

    print(detect)
    cv2.waitKey(1)
