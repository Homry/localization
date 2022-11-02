#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from handlers.videoHandler import VideoHandler
from handlers.coordsHandler import CoordsHandler
from handlers.markerHadler import MarkerHandler
from handlers.testGrid import testGrid
from utils.logger import Logger
from utils.configParser.config import Config
from grid.Point import Point


class LocalizeNode:
    def __init__(self):
        rospy.init_node('localize_node', log_level=rospy.INFO)

        camera = rospy.get_param('~camera')
        self.logger = Logger(f'Localization {camera}')

        self.config = Config.fromfile(rospy.get_param('~config'))['conf'][camera]
        self.logger.info(f'init config {self.config}')
        self.video = VideoHandler(camera, self.config)
        self._markerHandler = MarkerHandler(self.config, [349])
        tmp_pts = np.float32([[570, 1322], [570, 570], [1507, 570], [1507, 1322]])
        self._coordsHandler = CoordsHandler(self.config, self.video.getImageShape(), tmp_pts)
        self.testGrid = testGrid()
        self.logger.info(f'successfully init {camera} localize_node')
        self._publisher = rospy.Publisher(
            "robot_pose", Vector3, queue_size=1
        )

    def localize(self):
        image = self.video.getImage()
        robots_coords = self._coordsHandler.getWrapperCoords(self._markerHandler.getRobotCoords(image))
        for i in robots_coords:
            coords = self.testGrid.transform(Point(i[0], i[1]))
            coords = Point(coords[0], coords[1])
            msg_coords = Vector3(coords.getX(), coords.getY(), 0)
            self.logger.info(f'robot coords = {msg_coords}')
            self._publisher.publish(msg_coords)


if __name__ == "__main__":
    local = LocalizeNode()
    while not rospy.is_shutdown():
        local.localize()
