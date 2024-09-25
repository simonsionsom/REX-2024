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
cv2.moveWindow(WIN_RF, 50, 50)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

real_marker_height = 0.145

frame_center_x = imageSize[0] // 2
center_threshold = 350

grid_resolution = 0.05
grid_size = (100, 100)
occupancy_map = np.zeros(grid_size, dtype=bool)

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

def update_occupancy_map(center_x, center_y, distance):
    grid_x = int(center_x / grid_resolution)
    grid_y = int(center_y / grid_resolution)
    radius = int(distance / grid_resolution)

    for i in range(max(0, grid_x - radius), min(grid_size[0], grid_x + radius + 1)):
        for j in range(max(0, grid_y - radius), min(grid_size[1], grid_y + radius + 1)):
            if (i - grid_x) ** 2 + (j - grid_y) ** 2 <= radius ** 2:
                occupancy_map[i, j] = True

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
                update_occupancy_map(center_x, center_y, distance)
                print(occupancy_map)
                print(occupancy_map_image)
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

    cv2.imshow(WIN_RF, image)
    occupancy_map_image = (occupancy_map * 255).astype(np.uint8)
    cv2.imshow("Simon L", occupancy_map_image)

cam.stop()
cv2.destroyAllWindows()
