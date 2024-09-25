import cv2
import numpy as np
import time

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Camera calibration parameters (These values are usually obtained through a camera calibration process)
# Example camera matrix and distortion coefficients
camera_matrix = np.array([[1000, 0, 640],
                          [0, 1000, 360],
                          [0, 0, 1]], dtype=float)

dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion for simplicity

# Open a camera device for capturing
imageSize = (1280, 720)
FPS = 30
marker_size = 0.145  # Known real-world size of the marker in meters (14.5 cm)
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

while cv2.waitKey(4) == -1:  # Wait for a key press
    # Capture frame-by-frame from the picamera
    image = cam.capture_array("main")

    # Convert the image to grayscale (ArUco detection works better in grayscale)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect ArUco markers in the grayscale image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected, estimate the pose
    if ids is not None:
        # Draw detected markers on the image
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # Estimate the pose of each marker using estimatePoseSingleMarkers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        # Loop through all detected markers and draw the axes
        for i in range(len(ids)):
            # Draw the 3D axis for each marker
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)

            # Extract the rotation and translation vectors
            rvec = rvecs[i]
            tvec = tvecs[i]

            # Display the translation vector (distance) on the image
            distance = np.linalg.norm(tvec)  # Calculate distance from the camera to the marker
            cv2.putText(image, f"ID: {ids[i][0]} Dist: {distance:.2f}m",
                        (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Print marker ID, rotation vector, and translation vector (distance) in the console
            print(f"Marker ID: {ids[i][0]}")
            print(f"Rotation Vector: {rvec}")
            print(f"Translation Vector: {tvec}")
            print(f"Distance: {distance:.2f} m")

    # Show the frame with detected markers and distance
    cv2.imshow(WIN_RF, image)

# Clean up after the loop
cam.stop()
cv2.destroyAllWindows()
