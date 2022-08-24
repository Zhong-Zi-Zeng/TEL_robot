from yolo_v4.Yolo_V4_Api import Detect
import cv2


network = Detect()

cap = cv2.VideoCapture(0)


while True:
    ret, img = cap.read()

    detect = network.detect_image(img)

    print(detect)
    cv2.waitKey(1)
