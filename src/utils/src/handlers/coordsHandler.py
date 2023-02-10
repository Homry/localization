import cv2
import numpy as np


class CoordsHandler:
    def __init__(self, config: dict, shape: tuple, pts: np.array):
        self._config = config
        #print(shape_)
        #shape = (int(shape_[1]), int(shape_[0]))
        self._optimalCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(config["camera_matrix"], config['dist_coefs'],
                                                                    shape, 1, shape)
        print(pts)
        pts = np.array([self._undistortPoints(i)[0][0] for i in pts])


        pts2 = np.float32([[pts[0, 0], pts[0, 1]],
                           [pts[0, 0], pts[2, 1]],
                           [pts[2, 0], pts[2, 1]],
                           [pts[2, 0], pts[0, 1]]])
        pts = np.float32(pts)
        self._perspectiveMatrix = cv2.getPerspectiveTransform(pts, pts2)

    def _undistortPoints(self, points: np.array) -> np.array:
        return cv2.undistortPoints(points, self._config["camera_matrix"], self._config['dist_coefs'], None,
                                   self._optimalCameraMatrix)

    def _perspectiveTransformPoints(self, points) -> np.array:
        return cv2.perspectiveTransform(points, self._perspectiveMatrix)

    def getWrapperCoords(self, marker_coords):
        res_coords = []
        for i in marker_coords:
            res_coords.append(self._perspectiveTransformPoints(self._undistortPoints(i))[0][0])
        return res_coords

    def undistortImage(self, image):
        return cv2.undistort(image, self._config["camera_matrix"], self._config['dist_coefs'], None,
                                   self._optimalCameraMatrix)

    def perspectiveTransformImage(self, image):
        return cv2.warpPerspective(image, self._perspectiveMatrix, (image.shape[1], image.shape[0]))
