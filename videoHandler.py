from typing import Tuple, Dict, Union
import numpy as np
import cv2
from utils import Logger
import apriltag
from grid import Point
from numba import jit, njit


class VideoHandler:
    def __init__(self, config, camera, Debug=True):

        self.config = config
        self.stream = cv2.VideoCapture(self.config['videoPath'])
        self.logger = Logger(f'VideoStream {camera}')
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self._pts = None
        self._debug = Debug
        self._optimalCameraMatrix = None
        self._perspectiveMatrix = None
        self._cols = None
        self._rows = None

    def _init_camera_matrix(self, image, pts):
        self._rows, self._cols = image.shape[:2]
        self._optimalCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(self.config["camera_matrix"],
                                                                     self.config['dist_coefs'],
                                                                     (self._cols, self._rows), 1,
                                                                     (self._cols, self._rows))
        pts2 = np.float32([[pts[0, 0], pts[0, 1]],
                           [pts[0, 0], pts[2, 1]],
                           [pts[2, 0], pts[2, 1]],
                           [pts[2, 0], pts[0, 1]]])
        pts = np.float32(pts)
        self._perspectiveMatrix = cv2.getPerspectiveTransform(pts, pts2)

    def _showImage(self, img):
        cv2.imshow('camera', img)
        if cv2.waitKey(1) == 27:
            exit(0)

    def getShape(self):
        status, image = self.stream.read()
        if status is not True:
            self.logger.error(f'getImage - {status}')
        return image.shape

    def _getImage(self) -> np.array:
        status, image = self.stream.read()

        if status is not True:
            self.logger.error(f'getImage - {status}')
        return image

    def getInitCameraState(self) -> Tuple[Dict, Dict, Tuple]:
        end = False
        virtual_config = {}
        real_conf = {}
        image = None
        while not end:
            image = self._getImage()
            markers_coords = self._getFloorCoords(image)
            if len(markers_coords) == 4:
                self._init_camera_matrix(image, markers_coords)
                self._pts = np.squeeze(self._undistortPoints(markers_coords))
                markers_coords = np.float32([[self._pts[0, 0], self._pts[0, 1]],
                                             [self._pts[0, 0], self._pts[2, 1]],
                                             [self._pts[2, 0], self._pts[2, 1]],
                                             [self._pts[2, 0], self._pts[0, 1]]])
                end = True
                index = 0
                for i in ["A", "B", "C", "D"]:
                    virtual_config[i] = Point(markers_coords[index][0], markers_coords[index][1])
                    index += 1
        index = 0
        for i in ["A", "B", "C", "D"]:
            real_conf[i] = Point(self.config['markers'][index][0], self.config['markers'][index][1])
            index += 1

        return virtual_config, real_conf, (image.shape[0], image.shape[1])

    def getWrappedImageWithRobotCoords(self) -> np.array:
        robot_coords = self._getRobotCoords(self._getImage())
        if len(robot_coords) == 0:
            return []

        robot_coords = self._perspectiveTransformPoints(self._undistortPoints(robot_coords))
        return robot_coords

    def _undistortPoints(self, points: np.array) -> np.array:
        print(points)
        return cv2.undistortPoints(points, self.config["camera_matrix"], self.config['dist_coefs'], None,
                                   self._optimalCameraMatrix)

    def _perspectiveTransformPoints(self, points) -> np.array:
        return cv2.perspectiveTransform(points, self._perspectiveMatrix)


    def _findAprilTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def _getRobotCoords(self, image):
        robot_tags = [349]
        robot_coords = []
        robot_id = []
        for i in self._findAprilTags(image):
            if i.tag_id in robot_tags:
                robot_coords.append(i.center)
                robot_id.append(i.tag_id)
        if self._debug:
            self.logger.debug(f'find robot Apriltag {robot_coords, robot_id}')
        return np.array(robot_coords)

    def _getFloorCoords(self, image):
        floor_coords = []
        floor_id = []
        for i in self._findAprilTags(image):
            if i.tag_id in self.config["markers_id"]:
                floor_coords.append(i.corners[0])
                floor_id.append(i.tag_id)
        if self._debug:
            self.logger.debug(f'find floor Apriltag {floor_coords, floor_id}')
        return np.array(floor_coords)

    def debug(self, image):
        end = False
        virtual_config = {}
        real_conf = {}
        index = 0
        for i in ["A", "B", "C", "D"]:
            virtual_config[i] = Point(self.config['pts'][index][0], self.config['pts'][index][1])
            index += 1
        index = 0
        for i in ["A", "B", "C", "D"]:
            real_conf[i] = Point(self.config['markers'][index][0], self.config['markers'][index][1])
            index += 1

        return virtual_config, real_conf, (image.shape[0], image.shape[1])
