#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def publish_image():
    while not rospy.is_shutdown():
        ret, img = cap.read()
        img = cv2.resize(img, (800, 600), interpolation=cv2.INTER_AREA)
        img_msgs = bridge.cv2_to_imgmsg(img, 'bgr8')
        pub.publish(img_msgs)

if __name__ == '__main__':
    rospy.init_node('image_publisher')
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    pub = rospy.Publisher('image_topic', Image, queue_size=1)
    rospy.loginfo('Start camera publisher...')
    publish_image()
