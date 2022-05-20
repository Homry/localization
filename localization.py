import time

from videoHandler import VideoHandler
from grid import Grid, Point
import cv2
from utils import Logger
from utils import Config
import matplotlib.pyplot as plt


class Localization:
    def __init__(self, configPath='./config/cameraConfig.py',
                 camera='camera1'):
        self.logger = Logger(f'Localization {camera}')
        self.config = Config.fromfile(configPath)['conf'][camera]
        self.logger.info(f'init config {self.config}')
        self.video = VideoHandler(self.config, camera)
        virtual, real, im_shape = self.video.get_init_camera_state()
        self.grid = Grid(real, virtual, im_shape, {"length": 10})
        self.x = 58.5/0.54
        self.y = 58.5/0.535
        self.map = cv2.imread('./map/autolab.jpeg')
        self.map = cv2.cvtColor(self.map, cv2.COLOR_BGR2RGB)

    def localize(self):
        image, robots_coords = self.video.get_wrapped_image_with_coords()
        robot_cords = []
        for i in robots_coords:
            coords = self.grid.get_coords(Point(i[0], i[1]))
            self.logger.info(f'robot coords = {coords.getX(), coords.getY()}')

            tmp = [coords.getX()*self.x, coords.getY()*self.y]
            robot_cords.append(tmp)
            self.logger.info(f'robot coords map = {tmp}')
        thickness = 2
        cords = (int(robot_cords[0][0]), int(robots_coords[0][1]))
        image_ = cv2.circle(self.map, cords, 5, (255, 0, 0), thickness)
        plt.imshow('map', image_)
        plt.imshow('video', image)
        #cv2.startWindowThread()
        return image_, image




