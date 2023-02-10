#!/usr/bin/env python3
import cv2
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from localization_msgs.msg import RealRobotCoords


class MapNode:
    def __init__(self):
        rospy.init_node('map_node', log_level=rospy.DEBUG)
        self.map = cv2.imread(rospy.get_param('~map_path'))
        self.ratio_x = rospy.get_param('~ratio_x')
        self.ratio_y = rospy.get_param('~ratio_y')
        print('init')
        self._sub = rospy.Subscriber("/robot_pose", RealRobotCoords, self.createMapPoint, queue_size=5)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("map", Image)

    def createMapPoint(self, robot_point: RealRobotCoords):
        print(robot_point, 'printed!!!!!!')
        x = int(robot_point.coords.x*self.ratio_x)
        y = int(robot_point.coords.y*self.ratio_y)

        center = (x, self.map.shape[0] - y)

        self.map = cv2.circle(self.map, center, 1, (0, 0, 255), 2)
        image_message = self.bridge.cv2_to_imgmsg(self.map, encoding="bgr8")
        self.image_pub.publish(image_message)


if __name__ == "__main__":
    map = MapNode()
    rospy.spin()