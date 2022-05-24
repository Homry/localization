import numpy as np
from grid import Point
import cv2


class Square:
    def __init__(self, x, y, length):

        self._virtual_a = Point(x*length, y*length+length)
        self._virtual_b = Point(x*length, y*length)
        self._virtual_c = Point(x*length+length, y*length)
        self._virtual_d = Point(x*length+length, y*length+length)

        self._real_a = None
        self._real_b = None
        self._real_c = None
        self._real_d = None

    def getA(self):
        return self._virtual_a

    def getB(self):
        return self._virtual_b

    def getC(self):
        return self._virtual_c

    def getD(self):
        return self._virtual_d

    def createRealCoords(self, virtual, ratio_x, ratio_y):

        self._real_a = Point(virtual.getA().getX()*ratio_x, virtual.getA().getY()*ratio_y)
        self._real_b = Point(virtual.getB().getX()*ratio_x, virtual.getB().getY()*ratio_y)
        self._real_c = Point(virtual.getC().getX()*ratio_x, virtual.getC().getY()*ratio_y)
        self._real_d = Point(virtual.getD().getX()*ratio_x, virtual.getD().getY()*ratio_y)

    def getRobotCoords(self):
        return self._real_a + (self._real_b-self._real_a)/2

    def __repr__(self):
        return f'virtual: {self._virtual_a, self._virtual_b, self._virtual_c, self._virtual_d}, \n real: {self._real_a, self._real_b, self._real_c, self._real_d}'




    def debug(self, image):
        return cv2.rectangle(image, self._virtual_b.getPoint(), self._virtual_d.getPoint(), (255, 0, 0), 2)

