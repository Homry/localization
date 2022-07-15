from typing import Tuple, Dict, Union
import numpy as np
import cv2
from utils import Logger
import apriltag
from grid import Point


class VideoHandler:
    def __init__(self, config, camera, Debug=True):

        self.config = config
        self.stream = cv2.VideoCapture(self.config['videoPath'])
        self.logger = Logger(f'VideoStream {camera}')
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self._pts = config['pts']
        self._debug = Debug

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
            image = self._undistort(self._getImage())
            markers_coords = self._getFloorCoords(image)
            if len(markers_coords) == 4:
                self._pts = markers_coords
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

    def getWrappedImageWithRobotCoords(self) -> Tuple[np.array, np.array]:
        image = self._undistort(self._getImage())
        floor_cords = self._getFloorCoords(image)
        image = self._warpPerspective(image, floor_cords)
        robot_coords = self._getRobotCoords(image)
        return image, robot_coords

    def _warpPerspective(self, img, pts1):
        rows, cols = img.shape[:2]
        if len(pts1) == 4:
            self._pts = pts1
        pts2 = np.float32([[self._pts[0, 0], self._pts[0, 1]],
                           [self._pts[0, 0], self._pts[2, 1]],
                           [self._pts[2, 0], self._pts[2, 1]],
                           [self._pts[2, 0], self._pts[0, 1]]])

        H = cv2.getPerspectiveTransform(pts2, self._pts)
        img = cv2.warpPerspective(img, H, (cols, rows), flags=cv2.WARP_INVERSE_MAP)
        if not self._debug:
            cv2.imshow('camera', img)
            if cv2.waitKey(1) == 27:
                exit(0)
        return img

    def _undistort(self, img) -> np.array:
        rows, cols = img.shape[:2]
        newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(self.config["camera_matrix"], self.config['dist_coefs'],
                                                           (cols, rows), 1, (cols, rows))
        return cv2.undistort(img, self.config["camera_matrix"], self.config['dist_coefs'], None, newCameraMatrix)

    def _findAprilTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def _getRobotCoords(self, image):
        robot_tags = [419]
        robot_coords = []
        robot_id = []
        for i in self._findAprilTags(image):
            if i.tag_id in robot_tags:
                robot_coords.append(i.center)
                robot_id.append(i.tag_id)
        if self._debug:
            self.logger.debug(f'find robot Apriltag {robot_coords, robot_id}')
        return robot_coords

    def _getFloorCoords(self, image):
        floor_coords = []
        floor_id = []
        for i in self._findAprilTags(image):

            if i.tag_id in self.config["markers_id"]:
                #        self.logger.debug(f'{[i]}')
                floor_coords.append(i.corners[0])
                floor_id.append(i.tag_id)
        if self._debug:
            self.logger.debug(f'find floor Apriltag {floor_coords, floor_id}')
        return np.float32(floor_coords)

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
