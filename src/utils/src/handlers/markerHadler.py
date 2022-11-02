import cv2
import apriltag
import numpy as np


def getCoords(markers, markers_id: list, all=False):
    save_id = []
    markers_coords = []
    for i in markers:
        if i.tag_id in markers_id:
            markers_coords.append(i.corners[0]) if all == False else markers_coords.append(i.corners)
            save_id.append(i.tag_id)
    return markers_coords, save_id


class MarkerHandler:
    def __init__(self, floor_markers: np.array, robots_markers: list):
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self._robots_markers_id = robots_markers
        self._floor_markers_id = floor_markers

    def _findAprilTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def getRobotCoords(self, image):
        robot_coords, robot_id = getCoords(self._findAprilTags(image), self._robots_markers_id)
        return np.array(robot_coords)

    def getFloorCoords(self, image):
        floor_coords, floor_id = getCoords(self._findAprilTags(image), self._floor_markers_id)
        return np.array(floor_coords)

    def getAll(self, image):
        robot_coords, robot_id = getCoords(self._findAprilTags(image), self._floor_markers_id, True)
        return np.array(robot_coords)
