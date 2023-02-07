#!/usr/bin/env python3
import cv2
import rospy
import apriltag
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from utils.configParser.config import Config
from localization_msgs.msg import FloorCoords, RobotCoords, Image as localImage


def getCoords(markers, markers_id: list, center=False):
    markers_coords = []
    save_id = []
    center_coords = []
    for i in markers:
        if i.tag_id in markers_id:
            markers_coords.append(i.corners)
            center_coords.append(i.center)
            #markers_coords.append(i.corners[3]) if center == False else markers_coords.append(i.center)
            save_id.append(i.tag_id)
    if center:
        return markers_coords, save_id, center_coords
    else:
        return markers_coords, save_id


def update_crop_coords(coords, update_cords):
    coords = [[coords[i][0] + update_cords[i][0], coords[i][1] + update_cords[i][1]] for i in
              range(len(coords))]
    top_left_x = int(min([i[0] for i in coords]))
    top_left_y = int(min([i[1] for i in coords]))
    bot_right_x = int(max([i[0] for i in coords]))
    bot_right_y = int(max([i[1] for i in coords]))

    top_left_x = 0 if top_left_x < 0 else top_left_x
    top_left_y = 0 if top_left_y < 0 else top_left_y
    bot_right_x = 0 if bot_right_x < 0 else bot_right_x
    bot_right_y = 0 if bot_right_y < 0 else bot_right_y
    return [top_left_y, bot_right_y, top_left_x, bot_right_x]


class FindApriltagNode:
    def __init__(self):
        rospy.init_node('find_apriltag_node', log_level=rospy.DEBUG)
        camera = rospy.get_param('~camera')
        print(camera)
        self._floor_markers_id = Config.fromfile(rospy.get_param('~config'))['conf'][camera]['markers_id']
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self._robots_markers_id = [349]
        self.bridge = CvBridge()
        self._sub = rospy.Subscriber(f"/{camera}/Image", localImage, self.find, queue_size=5)
        self._robot_publisher = rospy.Publisher(
            f"{camera}/debug/findApriltag", Image, queue_size=1
        )
        self._robot_cords_publisher = rospy.Publisher(
            f"{camera}/robot_cords", RobotCoords, queue_size=1
        )
        self.prev_exist = False
        self.prev_state = None
        self.send_floor = False
        self._floor_coords_publisher = rospy.Publisher(f'{camera}/floorCoords', FloorCoords, queue_size=1)

        self.update_cords = [[-100, -100], [100, -100], [-100, 100], [100, 100]]

    def send_floor_coords(self, coords, shape):
        msg = FloorCoords()
        msg.coords = [Vector3(i[0], i[1], 0) for i in coords]
        msg.shape = Vector3(shape[0], shape[1], shape[2])
        self._floor_coords_publisher.publish(msg)

    def find(self, msg: localImage):
        image = self.bridge.imgmsg_to_cv2(msg.image)
        if not self.send_floor:
            self.send_floor = True
            markers = self.getFloorCoords(image)
            if len(markers) == 4:
                self.send_floor_coords(markers, image.shape)

        if not self.prev_exist:

            coords, center = self.getRobotCoords(image)
            center = center[0] if len(center) > 0 else []
            coords = coords[0] if len(coords) > 0 else []

            if len(coords) > 0:
                coords = update_crop_coords(coords, self.update_cords)
                self.prev_exist = True
                self.prev_state = [coords[0], coords[1], coords[2], coords[3]]

        else:
            image = image[self.prev_state[0]:self.prev_state[1], self.prev_state[2]:self.prev_state[3]]

            coords, center = self.getRobotCoords(image)
            coords = coords[0] if len(coords) > 0 else []
            coords = [[i[0]+self.prev_state[2], i[1]+self.prev_state[0]] for i in coords]

            center = center[0] if len(center) > 0 else []
            center = [center[0]+self.prev_state[2], center[1]+self.prev_state[0]] if len(center) > 0 else []

            if len(coords) == 0:
                self.prev_exist = False
                self.prev_state = None
            else:
                self.prev_state = update_crop_coords(coords, self.update_cords)

        if len(center) > 0:
            coords_msg = RobotCoords()
            coords_msg.coords = [Vector3(center[0], center[1], 0)] if len(center) > 0 else []
            coords_msg.time = msg.time
            self._robot_cords_publisher.publish(coords_msg)

        image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self._robot_publisher.publish(image_message)

    def _findAprilTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def getRobotCoords(self, image):
        robot_coords, saved, center = getCoords(self._findAprilTags(image), self._robots_markers_id, center=True)
        return np.array(robot_coords), np.array(center)

    def getFloorCoords(self, image):
        floor_coords, saved_id = getCoords(self._findAprilTags(image), self._floor_markers_id)
        return np.array(floor_coords)

    def getAll(self, image):
        robot_coords, robot_id = getCoords(self._findAprilTags(image), self._floor_markers_id, True)
        return np.array(robot_coords)


if __name__ == "__main__":
    map = FindApriltagNode()
    rospy.spin()
