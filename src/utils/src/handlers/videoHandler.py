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

    def getImageShape(self) -> tuple:
        status, image = self._stream.read()
        return image.shape[:2]


if __name__ == "__main__":
    config = {
        "camera_matrix": np.float32([[2729.490263990151, 0.0, 1831.9202044872143],
                                     [0.0, 2733.96527631694, 948.9345546488366],
                                     [0.0, 0.0, 1.0]]),
        "dist_coefs": np.float32(
            [-0.3540047849355193, 0.11021530240142789, -0.007400920530526965, -0.008230929654647412, 0.0]),
        #"videoPath": "http://autolab.moevm.info/camera_4/live.mjpg",
        "videoPath": "rtsp://admin:@10.135.4.235/trackID=1",
        "rectification_matrix": np.float32([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]]),
        "projection_matrix": np.float32([[2126.650390625, 0.0, 1702.0329726830823, 0.0],
                                         [0.0, 2549.162841796875, 918.8533423289846, 0.0],
                                         [0.0, 0.0, 1.0, 0.0]]),
        "pts": np.float32([[332, 659], [399, 377], [699, 373], [673, 656]]),
        "markers": np.array([[0.6355, 0.6165], [0.6355, 1.413], [1.416, 1.42], [1.416, 0.6165]]),
        "markers_id": np.array([300, 301, 302, 303])

    }
    video = VideoHandler('camera4', config)
    while True:
        video.showImage(video.getImage())







