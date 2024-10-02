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
intrinsic_matrix = np.asarray([ 1760, 0, 640, 
                                0, 1760, 360, 
                                0, 0, 1. ], dtype = np.float64)

intrinsic_matrix.shape = (3, 3)

# distortion_coeffs = np.asarray([3.37113443e+00, -5.84490229e+01,
#        -9.99698589e-02, -2.84566227e-02, 1.18763855e+03], dtype = np.float64)
distortion_coeffs = np.array[0,0,0,0,0]
def draw_aruco_objects(image):
    """Draws detected objects and their orientations on the image given in img."""
    if not isinstance(ids, type(None)):
        outimg = cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(ids.shape[0]):
            outimg = cv2.drawFrameAxes(outimg, intrinsic_matrix,
                                        rvecs[i], tvecs[i], real_marker_height)
    else:
        outimg = image

    return outimg
while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, real_marker_height, intrinsic_matrix, distortion_coeffs)
    if ids is not None:
        for i in range(len(ids)):
            print("Object ID = ", ids[i], ", Distance = ", tvecs[i], ", angles = ", rvecs[i])
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
    print(rvecs)
    resized_image = cv2.resize(image, (320, 240))
    cv2.setWindowProperty(WIN_RF, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.imshow(WIN_RF, resized_image)

cam.stop()
cv2.destroyAllWindows()