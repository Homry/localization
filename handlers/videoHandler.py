import cv2
import numpy as np
from utils import Logger


class VideoHandler:
    def __init__(self, camera: str, config: dict):
        self._stream = cv2.VideoCapture(config['videoPath'])
        self._camera = camera
        self.logger = Logger(f'VideoStream {camera}')

    def getImage(self) -> np.array:
        status, image = self._stream.read()
        if status is not True:
            self.logger.error(f'getImage - {status}')
            raise RuntimeError('error with image from video stream')
        return image

    def showImage(self, image: np.array = None) -> None:
        if image is None:
            image = self.getImage()
        cv2.imshow(self._camera, image)
        if cv2.waitKey(1) == 27:
            exit(0)





