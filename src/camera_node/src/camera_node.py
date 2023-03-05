#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from localization_msgs.msg import Image as localImage
from utils.configParser.config import Config


class CameraNode:
    def __init__(self):

        rospy.init_node(f'camera_node', log_level=rospy.INFO)
        camera = rospy.get_param('~camera')
        self.camera = camera
        self.config = Config.fromfile(rospy.get_param('~config'))['conf'][camera]['videoPath']
        self._stream = cv2.VideoCapture(self.config)
        self._publisher_debug = rospy.Publisher(
            f"{camera}/debug/Image/", Image, queue_size=1
        )
        self._publisher = rospy.Publisher(
            f"{camera}/Image", localImage, queue_size=1
        )
        self.bridge = CvBridge()
        self.__counter = 0

    def stream(self):
        status, image = self._stream.read()
        time = rospy.get_time()
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image = localImage(time=time, image=image_message)

        self._publisher.publish(image)
        self._publisher_debug.publish(image_message)
        print(f'{self.camera} read')
        self.__counter = 0



if __name__ == '__main__':
    camera = CameraNode()
    #rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        camera.stream()



