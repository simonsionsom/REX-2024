import cv2
import numpy as np
import time

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

import robot  # Import your robot module

# Create a robot object and initialize
arlo = robot.Robot()

# Define the speed for rotation and movement
leftSpeed = 32
rightSpeed = 32
forwardSpeed = 48  # Speed for driving forward

# Define the minimum distance threshold (in meters) to stop the robot
min_distance = 0.3  # Stop if closer than 30 cm to the marker

# Open a camera device for capturing
imageSize = (1280, 720)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds
# Frame size
frame_center_x = imageSize[0] // 2
center_threshold = 350  # Threshold to consider the marker centered
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                queue=False)
cam.configure(picam2_config)
cam.start(show_preview=False)

time.sleep(1)  # wait for the camera to setup

# Open a window
WIN_RF = "Aruco Marker Detection"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Camera calibration parameters (These are just placeholders, use actual calibration data)
# You need to calibrate your camera or use predefined parameters if known
camera_matrix = np.array([[1000, 0, imageSize[0] // 2],
                          [0, 1000, imageSize[1] // 2],
                          [0, 0, 1]], dtype=float)

dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion, replace with actual coefficients if available

marker_length = 0.145  # Real marker size in meters (14.5 cm)

# Robot control functions (using your current rotate function)
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

while cv2.waitKey(4) == -1:  # Wait for a key press
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect ArUco markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected, estimate the pose
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            # Draw the pose of the marker
            cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)  # Draw axis with a length of 10 cm

            # Extract translation vector (tvecs) for the marker
            distance = np.linalg.norm(tvecs[i][0])  # Distance to the marker
            center_x, center_y = corners[i][0].mean(axis=0)  # Marker center in image coordinates

            # Display the distance on the image
            cv2.putText(image, f"Distance: {distance:.2f} m", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m, Center: ({center_x:.2f}, {center_y:.2f})")

            # Check if the marker is centered
            if abs(center_x - frame_center_x) > center_threshold:
                rotate_robot()
            else:
                stop_rotation()
                if distance < min_distance:
                    stop_robot()
                else:
                    drive_forward()

    else:
        # No markers detected, rotate the robot
        rotate_robot()

    resized_image = cv2.resize(image, (320, 240))  # Resize to a smaller size
    cv2.imshow(WIN_RF, resized_image)

# Clean up
cam.stop()
cv2.destroyAllWindows()
