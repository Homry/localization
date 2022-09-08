import time

import cv2
import numpy as np
from markerHadler import MarkerHandler
from coordsHandler import CoordsHandler
import apriltag

camera = {
    "camera_matrix": np.float32([[2729.490263990151, 0.0, 1831.9202044872143],
                                 [0.0, 2733.96527631694, 948.9345546488366],
                                 [0.0, 0.0, 1.0]]),
    "dist_coefs": np.float32(
        [-0.3540047849355193, 0.11021530240142789, -0.007400920530526965, -0.008230929654647412, 0.0]),
    # "videoPath": "http://autolab.moevm.info/camera_4/live.mjpg",
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

if __name__ == "__main__":
    detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
    markersHandler = MarkerHandler(camera["markers_id"], [])
    image_ = cv2.imread('./testImage.png')

    rows, cols = image_.shape[:2]
    t = time.time()
    newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(camera["camera_matrix"], camera['dist_coefs'],
                                                       (cols, rows), 1, (cols, rows))
    image = cv2.undistort(image_, camera["camera_matrix"], camera['dist_coefs'], None, newCameraMatrix)
    pts = np.float32([[588, 1327], [761, 634], [1452, 621], [1417, 1321]])
    pts2 = np.float32([[pts[0, 0], pts[0, 1]],
                       [pts[0, 0], pts[2, 1]],
                       [pts[2, 0], pts[2, 1]],
                       [pts[2, 0], pts[0, 1]]])
    H = cv2.getPerspectiveTransform(pts2, pts)
    img = cv2.warpPerspective(image, H, (cols, rows), flags=cv2.WARP_INVERSE_MAP)
    coords_after_manual = markersHandler.getFloorCoords(img)
    t = time.time() - t
    print(f'manual = {t}')

    coordsHandler = CoordsHandler(camera, image_.shape[:2], pts)
    t1 = time.time()
    coords_after_handler = markersHandler.getFloorCoords(image_)
    coords_after_handler = coordsHandler.getWrapperCoords(coords_after_handler)
    t1 = time.time()-t1
    print(f'handler = {t1}')
    print(f'dif = {t - t1}, div = {t/t1}')
    #print(coords_after_handler)


    #print(coords_after_manual)
    x_err = []
    y_err = []

    for i, coords in enumerate(coords_after_handler):
        x_err.append(abs(coords[0]-coords_after_manual[i][0])/coords[0]*100)
        y_err.append(abs(coords[1] - coords_after_manual[i][1]) / coords[1] * 100)
    print(f'x_err = {x_err},\ny_err = {y_err}')
    x_err = np.array(x_err).mean()
    y_err = np.array(y_err).mean()
    print(f'x_err = {x_err}, y_err = {y_err}')
