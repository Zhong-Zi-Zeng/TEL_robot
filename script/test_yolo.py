from First_Stage.Yolo_V4_Api import Detect
import cv2

cap = cv2.VideoCapture(1)
yolo = Detect()

while True:
    ret, img = cap.read()

    detections = yolo.detect_image(img)
    print(detections)

    cv2.imshow('', img)
    cv2.waitKey(1)
