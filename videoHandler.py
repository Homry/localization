from typing import Tuple, Dict
import numpy as np
import cv2
from utils import Logger
import apriltag
from grid import Point


class VideoHandler:
    def __init__(self, config, camera, Debug=False):

        self.config = config
        self.stream = cv2.VideoCapture(self.config['videoPath'])
        self.logger = Logger(f'VideoStream {camera}')
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self._pts = self.config["pts"]
        self._debug = False

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

    def get_init_camera_state(self) -> Tuple[Dict, Dict, Tuple]:
        end = False
        virtual_config = {}
        real_conf = {}
        image = None
        while not end:
            image = self._undistort(self._getImage())
            markers_coords, robot_coords = self._findAprilTags(image)
            if len(markers_coords) == 4:
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

    def get_wrapped_image_with_coords(self) -> Tuple[np.array, np.array]:
        image = self._undistort(self._getImage())
        markers_coords, robot_coords = self._findAprilTags(image)
        image = self._Homo(image, markers_coords)
        markers_coords, robot_coords = self._findAprilTags(image)
        return image, robot_coords

    def _Homo(self, img, pts1):
        rows, cols = img.shape[:2]
        if len(pts1) == 4:
            self._pts = pts1
        if self._debug:
            self._pts = self.config["pts"]
        pts2 = np.float32([[self._pts[0, 0], self._pts[0, 1]],
                           [self._pts[0, 0], self._pts[2, 1]],
                           [self._pts[2, 0], self._pts[2, 1]],
                           [self._pts[2, 0], self._pts[0, 1]]])

        H = cv2.getPerspectiveTransform(pts2, self._pts)
        img = cv2.warpPerspective(img, H, (cols, rows), flags=cv2.WARP_INVERSE_MAP)

        return img

    def _undistort(self, img) -> np.array:
        rows, cols = img.shape[:2]
        newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(self.config["camera_matrix"], self.config['dist_coefs'],
                                                           (cols, rows), 1, (cols, rows))
        return cv2.undistort(img, self.config["camera_matrix"], self.config['dist_coefs'], None, newCameraMatrix)

    def _findAprilTags(self, image) -> Tuple[np.float32, np.array]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        tags = self.config["markers_id"]
        robot_tags = [419]
        res = []
        robot = []
        find_tag = []
        for i in results:
            if i.tag_id in tags:
                res.append(i.center)
                find_tag.append(i.tag_id)
        for i in results:
            if i.tag_id in robot_tags:
                robot.append(i.center)
                find_tag.append(i.tag_id)
        self.logger.debug(f'findApriltag {res, find_tag}')
        return np.float32(res), np.array(robot)