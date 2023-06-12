from typing import Dict, Tuple
from grid import Square, Point
import numpy as np
import cv2


class Grid:
    def __init__(self, shape, step, real, virtual):
        print(shape)
        if shape[0] % step != 0 or shape[1] % step != 0:
            raise Exception('The dimension of the image does not match the step')
        self.shape = shape
        self._step = step
        self._grid = []
        real_a = real["A"]
        real_b = real["B"]
        real_d = real["D"]
        virtual_a = virtual["A"]
        virtual_b = virtual["B"]
        virtual_d = virtual["D"]

        real_length_x = np.sqrt(np.square(real_d.getX() - real_a.getX()) + np.square(real_d.getY() - real_a.getY()))
        real_length_y = np.sqrt(np.square(real_b.getX() - real_a.getX()) + np.square(real_b.getY() - real_a.getY()))
        virtual_length_x = np.sqrt(np.square(virtual_d.getX() - virtual_a.getX()) + np.square(virtual_d.getY() -
                                                                                              virtual_a.getY()))
        virtual_length_y = np.sqrt(np.square(virtual_b.getX() - virtual_a.getX()) + np.square(virtual_b.getY() -
                                                                                              virtual_a.getY()))
        ratio_x = real_length_x / virtual_length_x
        ratio_y = real_length_y / virtual_length_y

        for j in range(shape[0]//step):
            tmp = []
            for i in range(shape[1]//step):
                tmp.append(Square(i, j, step))
            self._grid.append(tmp)
        for i in range(shape[0]//step):
            for j in range(shape[1]//step):
                self._grid[i][j].createRealCoords(real, virtual, ratio_x, ratio_y)
        self._grid = np.array(self._grid)
        print(self._grid.shape)




    def getRobotCoords(self, virtual_coords):
        x = int(virtual_coords.getX() / self._step)
        y = int(virtual_coords.getY()/ self._step)
        return self._grid[int(y)][int(x)].getRobotCoords()

    def debug(self, image):
        for i in self._grid:
            for j in i:
                image = j.debug(image)
        return image







if __name__ == "__main__":
    image = cv2.imread('./frame.jpg')
    grid = Grid(image.shape, 10)
    image = grid.debug(image)
    cv2.imshow('debug', image)
    if cv2.waitKey(0) == 27:
        exit(0)

    print(image.shape)

