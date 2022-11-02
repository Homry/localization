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

    def __sub__(self, other):
        self._virtual_a = self._virtual_a-other.getA()
        self._virtual_b = self._virtual_b-other.getB()
        self._virtual_c = self._virtual_c-other.getC()
        self._virtual_d = self._virtual_d-other.getD()
        return self

    def createRealCoords(self, real, virtual, ratio_x, ratio_y):
        self._virtual_a -= virtual["A"]
        self._virtual_b -= virtual["B"]
        self._virtual_c -= virtual["C"]
        self._virtual_d -= virtual["D"]
        self._real_a = Point(real["A"].getX()+self._virtual_a.getX() * ratio_x, real["A"].getY()-self._virtual_a.getY() * ratio_y)
        self._real_b = Point(real["B"].getX()+self._virtual_b.getX() * ratio_x, real["B"].getY()-self._virtual_b.getY() * ratio_y)
        self._real_c = Point(real["C"].getX()+self._virtual_c.getX() * ratio_x, real["C"].getY()-self._virtual_c.getY() * ratio_y)
        self._real_d = Point(real["D"].getX()+self._virtual_d.getX() * ratio_x, real["D"].getY()-self._virtual_d.getY() * ratio_y)

    def getRobotCoords(self):
        return self._real_a + (self._real_b-self._real_a)/2

    def __repr__(self):
        return f'virtual: {self._virtual_a, self._virtual_b, self._virtual_c, self._virtual_d}, \n real: {self._real_a, self._real_b, self._real_c, self._real_d}'




    def debug(self, image):
        return cv2.rectangle(image, self._virtual_b.getPoint(), self._virtual_d.getPoint(), (255, 0, 0), 2)

