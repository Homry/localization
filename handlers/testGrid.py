from grid import Grid, Point
import cv2

from map.map import Map

if __name__ == "__main__":
    real = {
        "A": Point(0.541, 0.534),
        "B": Point(0.541, 1.601),
        "C": Point(2.171, 1.601),
        "D": Point(2.171, 0.534)
    }

    virt = {
        "A": Point(561, 1328),
        "B": Point(561, 570),
        "C": Point(1507, 570),
        "D": Point(1507, 1340)
    }
    map = Map('../map/autolab.jpeg', 57.5 / 0.54, 57.5 / 0.535)
    image = cv2.imread("undist.png")
    shape = image.shape[:2]
    print(shape)
    grid = Grid(shape, 5, real, virt)
    coords = []

    coords.append(grid.getRobotCoords(Point(128, 1707)))
    coords.append(grid.getRobotCoords(Point(323, 557)))
    print(f"coords = {coords}")
    for i in coords:
        map.createMapPoint(i)
