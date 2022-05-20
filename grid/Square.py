import numpy as np
from grid import Point


class Square:
    def __init__(self, virtual_x: int, virtual_y: int, length: int, ratio_x: np.float32, ratio_y: np.float32,
                 real_point: Point, virtual_point: Point):
        self._virtual_box = {
            "A": Point(virtual_x, virtual_y),
            "B": Point(virtual_x, virtual_y+length),
            "C": Point(virtual_x+length, virtual_y+length),
            "D": Point(virtual_x+length, virtual_y)
        }
        offset_x = (virtual_x - virtual_point.getX())*ratio_x
        offset_y = (virtual_y - virtual_point.getY())*-ratio_y
        length_x = length*ratio_x
        length_y = length*ratio_y
        A = Point(real_point.getX()+offset_x, real_point.getY()+offset_y)
        B = Point(real_point.getX()+offset_x, real_point.getY()+offset_y-length_y)
        C = Point(real_point.getX()+offset_x-length_x, real_point.getY()+offset_y-length_y)
        D = Point(real_point.getX()+offset_x-length_x, real_point.getY()+offset_y)
        self._real_box = {
            "A": A,
            "B": B,
            "C": C,
            "D": D
        }

    def get_coords(self) -> Point:
        center_x = self._real_box["A"].getX()
        center_y = self._real_box["A"].getY()
        return Point(center_x, center_y)

    def debug(self):
        return (self._virtual_box["A"].getX(), self._virtual_box["A"].getX()), (self._virtual_box["C"].getX(), self._virtual_box["C"].getX())

    def __repr__(self):
        return f'real: {self._real_box}, virtual: {self._virtual_box}'
