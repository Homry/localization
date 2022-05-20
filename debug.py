import time

from videoHandler import VideoHandler
from grid import Grid, Point
import numpy as np
import cv2
from utils import Logger
from utils import Config


class Debug:
    def __init__(self, configPath='./config/cameraConfig.py',
                 camera='camera1'):
        self.logger = Logger(f'Debug {camera}')
        self.config = Config.fromfile(configPath)['conf'][camera]
        self.logger.info(f'init config {self.config}')
        self.video = VideoHandler(self.config, camera, Debug=True)
        #virtual, real, im_shape = self.video.get_init_camera_state()
        virtual = {}
        index = 0
        for i in ["A", "B", "C", "D"]:
            virtual[i] = Point(self.config["pts"][index][0], self.config["pts"][index][1])
            index += 1

        real = {}
        index = 0
        for i in ["A", "B", "C", "D"]:
            real[i] = Point(self.config["markers"][index][0], self.config["markers"][index][1])
            index += 1
        imShape = self.video.getShape()
        im_shape = (imShape[0], imShape[1])
        self.grid = Grid(real, virtual, im_shape, {"length": 10})

    def localize(self):
        t = time.time()
        image, robots_coords = self.video.get_wrapped_image_with_coords()
        image = self.grid.debug(image)
        cv2.imshow("video", image)
        cv2.waitKey(0)

        '''for i in robots_coords:
            coords = self.grid.get_coords(Point(i[0], i[1]))
            self.logger.info(f'robot coords = {coords.getX(), coords.getY()}')
        print(time.time()-t)
        exit(1)'''



if __name__ == "__main__":
    local = Debug(camera="camera4")
    local.localize()
