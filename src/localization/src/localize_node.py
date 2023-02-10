#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from handlers.videoHandler import VideoHandler
from handlers.coordsHandler import CoordsHandler
from handlers.markerHadler import MarkerHandler
from localization_msgs.msg import FloorCoords, RobotCoords, RealRobotCoords
from handlers.testGrid import testGrid
from utils.logger import Logger
from utils.configParser.config import Config
from grid.Point import Point


class LocalizeNode:
    def __init__(self):
        rospy.init_node('localize_node', log_level=rospy.INFO)

        self.camera = rospy.get_param('~camera')
        self.logger = Logger(f'Localization {self.camera}')
        self.config = Config.fromfile(rospy.get_param('~config'))['conf'][self.camera]
        self.logger.info(f'init config {self.config}')

        self._floor_sub = rospy.Subscriber(f'{self.camera}/floorCoords', FloorCoords, self.init, queue_size=1)
        self._robot_sub = rospy.Subscriber(f'{self.camera}/robot_cords', RobotCoords, self.localize, queue_size=1)
        self._coordsHandler = None
        self.testGrid = None

        self._publisher = rospy.Publisher(
            f"robot_pose", RealRobotCoords, queue_size=1
        )

    def init(self, msg: FloorCoords):
        pts = np.array([[i.x, i.y] for i in msg.coords])
        self._coordsHandler = CoordsHandler(self.config, (int(msg.shape.x), int(msg.shape.y)), pts)
        virt_markers = self._coordsHandler.getWrapperCoords(pts)
        self.testGrid = testGrid(config=self.config, virtual_points=virt_markers)
        self.logger.info(f'successfully init {self.camera} localize_node')

    def localize(self, msg: RobotCoords):
        if self._coordsHandler is not None and self.testGrid is not None:
            coords = np.array([[msg.coords[0].x, msg.coords[0].y]])
            robots_coords = self._coordsHandler.getWrapperCoords(coords)
            for i in robots_coords:
                coords = self.testGrid.transform(Point(i[0], i[1]))
                coords = Point(coords[0], coords[1])
                msg_coords = RealRobotCoords(coords=Vector3(coords.getX(), coords.getY(), 0), time=msg.time)
                self.logger.info(f'robot coords = {msg_coords}')
                self._publisher.publish(msg_coords)


if __name__ == "__main__":
    local = LocalizeNode()
    rospy.spin()
