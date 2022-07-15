import os
from typing import Tuple, List, Any

import cv2

from grid import Grid, Point
from videoHandler import VideoHandler
from utils import Config

TWO_K = (2048, 1080)
CENTER = 'center'
CORNERS = 'corners'

X = 0
Y = 1

A = 0
B = 1
C = 2
D = 3

DATASET_DIR_IMG = "./datasets/images"
DATASET_DIR_LABEL = "./datasets/labels"


class VideoHandlerMarkers(VideoHandler):

    def __init__(self, **kwargs):
        super(VideoHandlerMarkers, self).__init__(**kwargs)

    def getWrappedImageWithRobotCoords(self):
        image = self._undistort(self._getImage())
        floor_cords = self._getFloorCoords(image)
        image = self._warpPerspective(image, floor_cords)
        return image, floor_cords

    def get_raw_markers(self, image):
        return self._findAprilTags(image)

    def test(self, show=True):
        image = self._getImage()  # self._undistort(self._getImage())
        results = self.get_raw_markers(image)
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (255, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 255), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))
            print("[INFO] tag id: {}".format(r.tag_id))
        # show the output image after AprilTag detection
        if show:
            cv2.imshow("Image", image)
            cv2.waitKey(0)
        else:
            return image

    def collect_data_img(self):
        image = self._getImage()
        results = self.get_raw_markers(image)
        markers: List[Any] = []

        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers

            # corners
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # center
            (cX, cY) = (int(r.center[0]), int(r.center[1]))

            tag_id = r.tag_id
            markers.append({
                "id": tag_id,
                "center": (cX, cY),
                "corners": (ptA, ptB, ptC, ptD)
            })

        return image, markers


def watch():
    video = get_video_instance()

    # A - UP LEFT
    # B - UP RIGHT
    # C - DOWN RIGHT
    # D - DOWN LEFT
    while True:
        video.test()


def save_img():
    video = get_video_instance()
    img = video.test(show=False)
    cv2.imwrite('../color_img.jpg', img)


def get_video_instance() -> VideoHandlerMarkers:
    config_path: str = 'config/cameraConfig.py'
    camera: str = 'camera4'
    config = Config.fromfile(config_path)['conf'][camera]
    video: VideoHandlerMarkers = VideoHandlerMarkers(config=config, camera=camera)
    return video


def normalize(length_x, length_y, max_x, max_y) -> Tuple[float, float]:
    return length_x / max_x, length_y / max_y


def save_dataset_element(data, label):
    # save img

    count_files = len(
        [name for name in os.listdir(DATASET_DIR_IMG) if os.path.isfile(os.path.join(DATASET_DIR_IMG, name))])
    print(count_files)
    cv2.imwrite(f"{DATASET_DIR_IMG}/img{count_files}.jpg", data)
    # save data
    data_str = '\n'.join([i for i in label])
    with open(f"{DATASET_DIR_LABEL}/img{count_files}.txt", "w+") as file:
        file.write(data_str)


def collect():
    video = get_video_instance()
    original_img, markers = video.collect_data_img()
    print(original_img.shape)
    height, width, _ = original_img.shape
    print(height, width)
    resized_image = cv2.resize(original_img, TWO_K, cv2.INTER_AREA)

    cv2.imwrite('../resized_image.jpg', resized_image)
    labels = []
    for marker in markers:
        norm_cx, norm_cy = normalize(marker[CENTER][X], marker[CENTER][Y], height, width)
        norm_w, norm_h = normalize(abs(marker[CORNERS][A][X] - marker[CORNERS][B][X]),
                                   abs(marker[CORNERS][A][Y] - marker[CORNERS][D][Y]),
                                   height, width
                                   )
        print(norm_cx, norm_cy, norm_w, norm_h)
        labels.append(f"0 {norm_cx} {norm_cy} {norm_w} {norm_h}")
    save_dataset_element(resized_image, labels)


if __name__ == '__main__':
    collect()
    # save_img()
    # watch()
    # image, robots_coords = video.getWrappedImageWithRobotCoords()
    # print(robots_coords)
    # print(image.shape)
