from typing import Dict, Tuple
from grid import Square, Point
import numpy as np
from logging import Logger
import cv2


class Grid:
    def __init__(self, real_config: Dict, virtual_config: Dict, image_shape: Tuple, grid_config: Dict):
        self._grid_config = grid_config
        if image_shape[0] % self._grid_config["length"] != 0 or image_shape[1] % self._grid_config["length"] != 0:
            raise Exception("WIP exception")
        self.logger = Logger(f'init Grid {real_config, virtual_config, image_shape, grid_config}')
        real_a = real_config["A"]
        real_b = real_config["B"]
        real_d = real_config["D"]
        virtual_a = virtual_config["A"]
        virtual_b = virtual_config["B"]
        virtual_d = virtual_config["D"]
        print(real_a, real_d)
        real_length_x = np.sqrt(np.square(real_d.getX() - real_a.getX()) + np.square(real_d.getY() - real_a.getY()))
        real_length_y = np.sqrt(np.square(real_b.getX() - real_a.getX()) + np.square(real_b.getY() - real_a.getY()))
        virtual_length_x = np.sqrt(np.square(virtual_d.getX() - virtual_a.getX()) + np.square(virtual_d.getY() -
                                                                                              virtual_a.getY()))
        virtual_length_y = np.sqrt(np.square(virtual_b.getX() - virtual_a.getX()) + np.square(virtual_b.getY() -
                                                                                              virtual_a.getY()))
        ratio_x = real_length_x / virtual_length_x
        ratio_y = real_length_y / virtual_length_y
        self._grid = []
        for i in range(int(image_shape[0]/self._grid_config["length"])):
            tmp = []
            for j in range(int(image_shape[1]/self._grid_config["length"])):
                tmp.append(Square(j * self._grid_config["length"], i * self._grid_config["length"],
                                          self._grid_config["length"], ratio_x, ratio_y, real_a, virtual_a))
            self._grid.append(tmp)


    def get_coords(self, virtual_coords: Point):
        x = np.round(virtual_coords.getX() / self._grid_config["length"])
        y = np.round(virtual_coords.getY() / self._grid_config["length"])
        print(x, y)
        return self._grid[int(y)][int(x)].get_coords()

    def debug(self, img):
        print(self._grid[0])
        test = np.array(self._grid)
        print(len(self._grid))
        print(len(self._grid[0]))
        for i in range(len(self._grid)):
            for j in range(len(self._grid[i])):
                A, C = self._grid[i][j].debug()
                color = (255, 0, 0)

                thickness = 2
                img = cv2.rectangle(img, A, C, color, thickness)
        return img

