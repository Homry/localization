import cv2

from grid import Point
import numpy as np
from scipy.linalg import solve

from handlers import VideoHandler, MarkerHandler, CoordsHandler
from map.map import Map
from utils import Config

'''if __name__ == "__main__":
    real = {
        "A": Point(0.541, 0.534),
        "B": Point(0.541, 1.601),
        "C": Point(1.628, 1.601),
        "D": Point(1.628, 0.534)
    }

    real_basis = {
        "A": Point(0.541-real["A"].getX(), 0.534-real["A"].getY()),
        "B": Point(0.541-real["A"].getX(), 1.601-real["A"].getY()),
        "C": Point(2.171-real["A"].getX(), 1.601-real["A"].getY()),
        "D": Point(1.628-real["A"].getX(), 0.534-real["A"].getY())
    }
    print(real_basis)

    virt = {
        "A": Point(570, 1322),
        "B": Point(570, 570),
        "C": Point(1507, 570),
        "D": Point(1507, 1322)
    }



    a11 = 937/1.087
    a12 = 0
    a21 = 0
    a22 = -752/1.067

    A = np.array([[a11, a12], [a21, a22]])
    A_1 = np.linalg.inv(A)

    tmp = -A_1@np.array([570, 1322])
    res = A_1@np.array([1006, 1714])+tmp + np.array([0.541, 0.534])
    #map = Map('./map/autolab.jpeg', 57.5 / 0.54, 57.5 / 0.535)
    #map.createMapPoint(Point(res[0], res[1]))
    print(res)



'''

if __name__ == "__main__":
    config = Config.fromfile('./config/cameraConfig.py')['conf']['camera4']
    video = VideoHandler("camera4", config)
    markerHandler = MarkerHandler(config['markers_id'], [349])
    tmp_pts = np.float32([[570, 1322], [570, 570], [1507, 570], [1507, 1322]])
    coordsHandler = CoordsHandler(config, video.getImageShape(), tmp_pts)
    img = video.getImage()
    markers = markerHandler.getAll(img)
    cv2.imshow('camera', img)
    if cv2.waitKey(1) == 27:
        pass
    for i in markers:
        rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(np.array(i, dtype=np.float32), np.float32(0.063), config['camera_matrix'], config['dist_coefs'])
        print(rvec, tvec)

