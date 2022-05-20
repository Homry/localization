class Point:
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def getX(self) -> int:
        return self._x

    def getY(self) -> int:
        return self._y

    def __repr__(self):
        return f'x={self._x}, y={self._y}'