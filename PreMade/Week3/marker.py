import cv2
import numpy as np
import time

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

import robot

arlo = robot.Robot()

imageSize = (1280, 720)
FPS = 30
focal_length = 1760
cam = picamera2.Picamera2()
frame_duration_limit = int(1 / FPS * 1000000)

picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                queue=False)
cam.configure(picam2_config)
cam.start(show_preview=False)

time.sleep(1)

WIN_RF = "Aruco Marker Detection"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

real_marker_height = 0.145

frame_center_x = imageSize[0] // 2
center_threshold = 350
intrinsic_matrix = np.asarray([ 6.0727040957659040e+02, 0., 3.0757300398967601e+02, 0.,
       6.0768864690145904e+02, 2.8935674612358201e+02, 0., 0., 1. ], dtype = np.float64)

intrinsic_matrix.shape = (3, 3)

distortion_coeffs = np.asarray([ 1.1911006165076067e-01, -1.0003366233413549e+00,
       1.9287903277399834e-02, -2.3728201444308114e-03, -2.8137265581326476e-01 ], dtype = np.float64)

def draw_aruco_objects(img, intrinsic_matrix, distortion_coeffs, rvecs, tvecs, real_marker_height):
    """Draws detected objects and their orientations on the image given in img."""
    if not isinstance(ids, type(None)):
        outimg = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        for i in range(ids.shape[0]):
            outimg = cv2.drawFrameAxes(outimg, intrinsic_matrix, distortion_coeffs,
                                        rvecs[i], tvecs[i], real_marker_height)
    else:
        outimg = img

    return outimg

while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, real_marker_height, intrinsic_matrix, distortion_coeffs)
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    print(rvecs)
    draw_aruco_objects
    resized_image = cv2.resize(image, (320, 240))
    cv2.setWindowProperty(WIN_RF, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.imshow(WIN_RF, resized_image)

cam.stop()
cv2.destroyAllWindows()
