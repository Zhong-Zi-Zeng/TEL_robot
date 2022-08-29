from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from collections import deque
import rospy

class ImageCallback:
    def __init__(self):
        # 建立柱列
        self.img_queue = deque(maxlen=1)

        # 初始化節點
        self.bridge = CvBridge()
        rospy.init_node('subscribe_image')
        rospy.Subscriber('/camera/color/image_raw', Image, self.img_callback)
        rospy.spin()

    def img_callback(self, img_msgs):
        img = self.bridge.imgmsg_to_cv2(img_msgs, 'bgr8')

        self.img_queue.append(img)

    def get_img(self):
        while len(self.img_queue) == 0:
            pass

        return self.img_queue.pop()

