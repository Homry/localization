from typing import Tuple


class Point:
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def getX(self) -> int:
        return self._x

    def getY(self) -> int:
        return self._y

    def getPoint(self) -> Tuple:
        return self._x, self._y

    def __add__(self, other):
        return Point(self._x + other.getX(), self._y + other.getY())

    def __sub__(self, other):
        return Point(self._x - other.getX(), self._y-other.getY())

    def __truediv__(self, other):
        return Point(self._x/other, self._y/other)

    def __repr__(self):
        return f'x={self._x}, y={self._y}'