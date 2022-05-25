import time

from videoHandler import VideoHandler
from grid import Grid, Point
import cv2
from utils import Logger
from utils import Config
from map.map import Map


class Localization:
    def __init__(self, configPath='./config/cameraConfig.py',
                 camera='camera1'):
        self.logger = Logger(f'Localization {camera}')
        self.config = Config.fromfile(configPath)['conf'][camera]
        self.logger.info(f'init config {self.config}')
        self.video = VideoHandler(self.config, camera)
        virtual, real, im_shape = self.video.getInitCameraState()
        self.grid = Grid(im_shape, 10, real, virtual)
        self.map = Map('./map/autolab.jpeg', 58.5/0.54, 58.5/0.535)


    def localize(self):
        image, robots_coords = self.video.getWrappedImageWithRobotCoords()
        for i in robots_coords:
            coords = self.grid.getRobotCoords(Point(i[0], i[1]))
            self.logger.info(f'robot coords = {coords.getX(), coords.getY()}')
            self.map.createMapPoint(coords)







