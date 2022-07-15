import cv2

from grid import Grid, Point
from videoHandler import VideoHandler
from utils import Config


class VideoHandlerMarkers(VideoHandler):
    # def __init__(self, configPath='./config/cameraConfig.py',
    #             camera='camera4'):
    #    self.config = Config.fromfile(configPath)['conf'][camera]
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
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
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
        # show the output image after AprilTag detection
        if show:
            cv2.imshow("Image", image)
            cv2.waitKey(0)
        else:
            return image


def watch():
    configPath = './config/cameraConfig.py'
    camera = 'camera4'
    config = Config.fromfile(configPath)['conf'][camera]
    video = VideoHandlerMarkers(config=config, camera=camera)

    # 0 -
    # 1 -
    # 2 -
    # 3 -
    while True:
        video.test()


def save_img():
    configPath = './config/cameraConfig.py'
    camera = 'camera4'
    config = Config.fromfile(configPath)['conf'][camera]
    video = VideoHandlerMarkers(config=config, camera=camera)
    img = video.test(show=False)
    cv2.imwrite('color_img.jpg', img)


if __name__ == '__main__':
    save_img()
        # image, robots_coords = video.getWrappedImageWithRobotCoords()
        # print(robots_coords)
    # print(image.shape)
