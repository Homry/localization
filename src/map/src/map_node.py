#!/usr/bin/env python3
import cv2
import rospy
from geometry_msgs.msg import Vector3


class MapNode:
    def __init__(self):
        rospy.init_node('map_node', log_level=rospy.INFO)
        self.map = cv2.imread(rospy.get_param('~map_path'))
        self.ratio_x = rospy.get_param('~ratio_x')
        self.ratio_y = rospy.get_param('~ratio_y')
        print('init')
        self._sub = rospy.Subscriber("/robot_pose", Vector3, self.createMapPoint, queue_size=1)

    def createMapPoint(self, robot_point: Vector3):
        print(robot_point)
        x = int(robot_point.x*self.ratio_x)
        y = int(robot_point.y*self.ratio_y)

        center = (x, self.map.shape[0] - y)

        self.map = cv2.circle(self.map, center, 1, (0, 0, 255), 2)
        cv2.imshow('map', self.map)
        if cv2.waitKey(1) == 27:
            pass


if __name__ == "__main__":
    map = MapNode()
    rospy.spin()