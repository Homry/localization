import numpy as np

conf = {
    "camera1": {
        "camera_matrix": np.float32([[1638.7601373492175, 0.0, 943.369937233313],
                                     [0.0, 1637.9847138215782, 608.8241424093957],
                                     [0.0, 0.0, 1.0]]),
        "dist_coefs": np.float32(
            [-0.5322013119088566, 0.26988003381355613, -0.02154990411267074, -0.005572796472868165, 0.0]),
        "videoPath": "http://autolab.moevm.info/camera_1/live.mjpg",
        "rectification_matrix": np.float32([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]]),
        "projection_matrix": np.float32([[1293.0950927734375, 0.0, 913.3648842158436, 0],
                                         [0.0, 1541.2896728515625, 603.059877065225, 0],
                                         [0.0, 0.0, 1.0, 0.0]]),
        "pts": np.float32([[300, 704], [440, 226], [780, 208], [732, 686]]),
        "test": np.float32([[237, 928], [301, 701], [515, 693], [477, 915]])

    },
    "camera2": {
        "camera_matrix": np.float32([[1340.872606020917, 0.0, 1004.9992813438527],
                                     [0.0, 1345.3591504235235, 498.63573897426164],
                                     [0.0, 0.0, 1.0]]),
        "dist_coefs": np.float32(
            [-0.36117581531932014, 0.11006966996112802, -0.0005395628699026254, -0.0014828076538022702, 0.0]),
        "videoPath": "http://autolab.moevm.info/camera_2/live.mjpg",
        "rectification_matrix": np.float32([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]]),
        "projection_matrix": np.float32([[1033.22265625, 0.0, 1023.6190501309466, 0.0],
                                         [0.0, 1257.5067138671875, 491.81431033084664, 0.0],
                                         [0.0, 0.0, 1.0, 0.0]]),
        "pts": np.float32([[765, 787], [802, 117], [900, 112], [980, 767]]),
        "test": np.float32([[340, 811], [399, 639], [573, 634], [548, 798]])

    },
    "camera3": {
        "camera_matrix": np.float32([[3086.47470366975, 0.0, 946.0661337407131],
                                     [0.0, 3036.568115548386, 584.6962530683811],
                                     [0.0, 0.0, 1.0]]),
        "dist_coefs": np.float32(
            [-1.809602866073375, 3.7064053114518574, -0.09713223713237157, -0.019529957288539943, 0.0]),
        "videoPath": "http://autolab.moevm.info/camera_3/live.mjpg",
        "rectification_matrix": np.float32([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]]),
        "projection_matrix": np.float32([[2565.9091796875, 0.0, 909.9803965089668, 0.0],
                                         [0.0, 2860.93310546875, 554.4865711588718, 0.0],
                                         [0.0, 0.0, 1.0, 0.0]]),
        "pts": np.float32([[321, 218], [360, 143], [1303, 163], [1312, 237]]),
        "test": np.float32([[272, 742], [329, 568], [525, 571], [482, 739]])

    },
    "camera4": {
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

    },
    "camera5": {
        "camera_matrix": np.float32([[1387.2568675735947, 0.0, 992.4798803587946],
                                     [0.0, 1388.4388188020225, 539.2257866164771],
                                     [0.0, 0.0, 1.0]]),
        "dist_coefs": np.float32(
            [-0.3880343644790106, 0.13972034087642235, -0.004806826164717015, -0.0009232652288272098, 0.0]),
        "videoPath": "http://autolab.moevm.info/camera_5/live.mjpg",
        "rectification_matrix": np.float32([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]]),
        "projection_matrix": np.float32([[1085.06884765625, 0.0, 1003.9939636930649, 0.0],
                                         [0.0, 1299.7818603515625, 534.6729534568367, 0.0],
                                         [0.0, 0.0, 1.0, 0.0]]),
        "pts": np.float32([[765, 870], [807, 260], [949, 255], [985, 870]]),
        "test": np.float32([[800, 367], [807, 260], [947, 260], [953, 362]])

    },
    "camera6": {
        "camera_matrix": np.float32([[1440.4395217760507, 0.0, 930.1558464523819],
                                     [0.0, 1440.1721143257955, 569.4715128090797],
                                     [0.0, 0.0, 1.0]]),
        "dist_coefs": np.float32(
            [-0.4213639841064852, 0.18352906859011683, -0.007967498067542303, -0.002056643837192403, 0.0]),
        "videoPath": "http://autolab.moevm.info/camera_6/live.mjpg",
        "rectification_matrix": np.float32([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]]),
        "projection_matrix": np.float32([[1143.59716796875, 0.0, 909.0048996878613, 0.0],
                                         [0.0, 1349.06005859375, 566.8213496592216, 0.0],
                                         [0.0, 0.0, 1.0, 0.0]]),
        "pts": np.float32([[736, 789], [771, 253], [906, 250], [943, 781]]),
        "test": np.float32([[771, 253], [775, 177], [896, 178], [902, 248]])

    }

}

#[array([441.52641783, 640.68346712]), array([494.23147511, 348.71006792]), array([723.42266441, 350.08795157]), array([693.10320855, 642.97000234])]

#2022-05-24 14:33:13,218 - VideoStream camera4 - DEBUG - find robot Apriltag ([array([1200.30051087,  544.68859987])], [419])
#2022-05-24 14:33:13,219 - Localization camera4 - INFO - robot coords = (2.940933123444532, 1.5256753937047676)
#2022-05-24 14:33:13,219 - Localization camera4 - INFO - robot coords map = [318.60108837315767, 166.82618790977364]