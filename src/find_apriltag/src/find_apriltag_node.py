#!/usr/bin/env python3
import cv2
import rospy
import apriltag
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.configParser.config import Config
from localization_msgs.msg import Image as localImage


def getCoords(markers, markers_id: list, center=False):
    markers_coords = []
    save_id = []
    for i in markers:
        if i.tag_id in markers_id:
            markers_coords.append(i.corners)
            #markers_coords.append(i.corners[3]) if center == False else markers_coords.append(i.center)
            save_id.append(i.tag_id)
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
        '''self._robot_cords_publisher = rospy.Publisher(
            f"{camera}/robot_cords"
        )'''
        self.prev_exist = False
        self.prev_state = None
        self.update_cords = [[-100, -100], [100, -100], [-100, 100], [100, 100]]

    def find(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg.image)
        if not self.prev_exist:

            coords = self.getRobotCoords(image)[0]
            coords = update_crop_coords(coords, self.update_cords)

            self.prev_exist = True
            self.prev_state = [coords[0], coords[1], coords[2], coords[3]]

        else:
            image = image[self.prev_state[0]:self.prev_state[1], self.prev_state[2]:self.prev_state[3]]
            try:
                coords = self.getRobotCoords(image)[0]
                coords = [[i[0]+self.prev_state[2], i[1]+self.prev_state[0]] for i in coords]
            except:
                coords = []
            if len(coords) == 0:
                self.prev_exist = False
                self.prev_state = None
            else:
                self.prev_state = update_crop_coords(coords, self.update_cords)

        image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self._robot_publisher.publish(image_message)

    def _findAprilTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def getRobotCoords(self, image):
        robot_coords, saved = getCoords(self._findAprilTags(image), self._robots_markers_id, center=True)
        return np.array(robot_coords)

    def getFloorCoords(self, image):
        floor_coords = getCoords(self._findAprilTags(image), self._floor_markers_id)
        return np.array(floor_coords)

    def getAll(self, image):
        robot_coords, robot_id = getCoords(self._findAprilTags(image), self._floor_markers_id, True)
        return np.array(robot_coords)


if __name__ == "__main__":
    map = FindApriltagNode()
    rospy.spin()
