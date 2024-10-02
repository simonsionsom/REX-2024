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

leftSpeed = 32
rightSpeed = 32
forwardSpeed = 48

min_distance = 0.3

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

def rotate_robot():
    print("Rotating robot...")
    arlo.go_diff(leftSpeed, rightSpeed, 1, 0)
    time.sleep(0.3)
    arlo.stop()
    time.sleep(0.2)

def stop_rotation():
    print("Stopping robot rotation...")
    arlo.stop()

def drive_forward():
    print("Driving forward...")
    arlo.go_diff(leftSpeed, rightSpeed, 1, 1)
    time.sleep(1)

def stop_robot():
    print("Stopping robot due to proximity...")
    arlo.stop()

while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            corner = corners[i][0]
            center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
            center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
            top_left_y = corner[0][1]
            bottom_left_y = corner[3][1]
            marker_height_in_pixels = abs(bottom_left_y - top_left_y)

            if marker_height_in_pixels > 0:
                distance = focal_length * (real_marker_height / marker_height_in_pixels)

                cv2.putText(image, f"Distance: {distance:.2f} m",
                            (int(corner[0][0]), int(corner[0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m, Center: ({center_x:.2f}, {center_y:.2f})")
                if abs(center_x - frame_center_x) > center_threshold:
                    rotate_robot()
                else:
                    stop_rotation()
                    if distance < min_distance:
                        stop_robot()
                    else:
                        drive_forward()
    else:
        rotate_robot()

    cv2.resizeWindow(WIN_RF, 500, 500)
    cv2.imshow(WIN_RF, image)
    resized_image = cv2.resize(image, (320, 240))
    cv2.resizeWindow(WIN_RF, 400, 400)
    cv2.setWindowProperty(WIN_RF, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.imshow(WIN_RF, resized_image)

cam.stop()
cv2.destroyAllWindows()
