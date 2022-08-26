from yolov4.darknet import darknet

class Detect:
    def __init__(self):
        self.config_file = '/home/heng/catkin_ws/src/TEL_robot/script/First_Stage/yolov4/yolov4-tiny-obj.cfg'
        self.weight_file = '/home/heng/catkin_ws/src/TEL_robot/script/First_Stage/yolov4/yolov4-tiny-obj_final.weights'
        self.data_file = '/home/heng/catkin_ws/src/TEL_robot/script/First_Stage/yolov4/TEL.data'

        # load network
        self.network, self.class_names, _\
            = darknet.load_network(self.config_file, self.data_file, self.weight_file, 1)

    def detect_image(self, img):
        # BGT to RGB
        img = img[:, :, ::-1]

        # Transform image type to darknet
        darknet_img = darknet.array_to_image(img)

        # Detect image
        detections = darknet.detect_image(self.network, self.class_names, darknet_img, thresh=0.5, nms=0.6)

        return detections
