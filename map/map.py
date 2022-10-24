import cv2


class Map:
    def __init__(self, map_path, ratio_x, ratio_y):
        self.map = cv2.imread(map_path)
        self.ratio_x = ratio_x
        self.ratio_y = ratio_y

    def createMapPoint(self, robot_point):
        x = int(robot_point.getX()*self.ratio_x)
        y = int(robot_point.getY()*self.ratio_y)

        center = (x,self.map.shape[0] - y)

        self.map = cv2.circle(self.map, center, 1, (0, 0, 255), 2)
        cv2.imshow('map', self.map)
        if cv2.waitKey(1) == 27:
            pass