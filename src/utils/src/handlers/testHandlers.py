import time

import cv2
import numpy as np

from grid import Point
from handlers import VideoHandler, testGrid
from markerHadler import MarkerHandler
from coordsHandler import CoordsHandler
from utils.configParser.config import Config
import apriltag



class LocalizeNode:
    def __init__(self):


        camera = 'camera5'


        self.config = Config.fromfile('../../../localization/config/cameraConfig.py')['conf'][camera]

        self.video = VideoHandler(camera, self.config)
        self._markerHandler = MarkerHandler(self.config['markers_id'], [349])

        tmp_pts = self._markerHandler.getFloorCoords(self.video.getImage())
        self._coordsHandler = CoordsHandler(self.config, self.video.getImageShape(), tmp_pts)
        virt_markers = []
        while len(virt_markers) != 4:
            image = self.video.getImage()
            virt_markers = self._coordsHandler.getWrapperCoords(self._markerHandler.getFloorCoords(image))
        self.testGrid = testGrid(config=self.config, virtual_points=virt_markers)


    def localize(self):
        image = self.video.getImage()
        robots_coords = self._coordsHandler.getWrapperCoords(self._markerHandler.getRobotCoords(image))
        print(f'transform coords = {robots_coords}')
        image = self._coordsHandler.perspectiveTransformImage(self._coordsHandler.undistortImage(image))
        cord = self._markerHandler.getRobotCoords(image)

        print(cord)
        floor = self._markerHandler.getFloorCoords(image)
        for i in floor:
            cord = (int(i[0]), int(i[1]))
            print(cord)
            image =  cv2.circle(image, cord, 1, (0, 0, 255), 2)
        cv2.imshow('test', image)
        key = cv2.waitKey(1)
        if key == ord("p"):
            cv2.waitKey(0)


if __name__ == "__main__":

    local = LocalizeNode()
    while True:
        local.localize()





