import numpy as np
import time

from handlers import VideoHandler, CoordsHandler, MarkerHandler, testGrid
from grid import Grid, Point

from utils import Logger
from utils import Config
from map.map import Map


class Localization:
    def __init__(self, configPath='./config/cameraConfig.py',
                 camera='camera1'):
        self.logger = Logger(f'Localization {camera}')
        self.config = Config.fromfile(configPath)['conf'][camera]
        self.logger.info(f'init config {self.config}')
        self.video = VideoHandler(camera, self.config)
        self._markerHandler = MarkerHandler(self.config, [349])
        tmp_pts = np.float32([[570, 1322], [570, 570], [1507, 570], [1507, 1322]])
        self._coordsHandler = CoordsHandler(self.config, self.video.getImageShape(), tmp_pts)
        self.testGrid = testGrid()
        self.map = Map('./map/autolab.jpeg', 57.5/0.54, 57.5/0.535)

    def localize(self):
        t = time.time()
        image = self.video.getImage()
        robots_coords = self._coordsHandler.getWrapperCoords(self._markerHandler.getRobotCoords(image))
        for i in robots_coords:
            coords = self.testGrid.transform(Point(i[0], i[1]))
            print(coords)
            coords = Point(coords[0], coords[1])
            self.logger.info(f'robot coords = {coords.getX(), coords.getY()}')
            self.map.createMapPoint(coords)
        print(time.time()- t)







