import cv2
import numpy as np
import time

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Open a camera device for capturing
imageSize = (1280, 720)
FPS = 30
focal_length = 2000  # Provided focal length
cam = picamera2.Picamera2()
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds

# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                queue=False)
cam.configure(picam2_config)  # Not really necessary
cam.start(show_preview=False)

time.sleep(1)  # wait for the camera to setup

# Open a window
WIN_RF = "Aruco Marker Detection"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Camera matrix using focal length and image center
fx = focal_length
fy = focal_length
cx = imageSize[0] / 2
cy = imageSize[1] / 2

# Camera intrinsic matrix
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])

# Distortion coefficients (assuming zero distortion)
dist_coeffs = np.zeros((4, 1))

# Marker size in meters (e.g., if markers are 10 cm, set to 0.1)
marker_length = 0.145

while cv2.waitKey(4) == -1:  # Wait for a key press
    # Capture frame-by-frame from the picamera
    image = cam.capture_array("main")

    # Convert the image to grayscale (ArUco detection works better in grayscale)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect ArUco markers in the grayscale image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected, estimate the pose
    if ids is not None:
        # Estimate pose of the markers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Draw detected markers on the image
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        for i in range(len(ids)):
            # Draw the axes for each marker
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)  # Axis length = 0.1m

            # Calculate the distance to the marker
            # Euclidean distance to the marker from the camera
            distance = np.linalg.norm(tvecs[i])

            # Alternatively, use only the z-axis value for an approximation
            z_distance = tvecs[i][0][2]

            # Display the distance on the image
            cv2.putText(image, f"Distance: {distance:.2f} m",
                        (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),  # Position of the text
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Print marker ID and distance in the console
            print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m (z-distance: {z_distance:.2f} m)")

    # Show the frame with detected markers and distance
    cv2.imshow(WIN_RF, image)

# Clean up after the loop
cam.stop()
cv2.destroyAllWindows()
