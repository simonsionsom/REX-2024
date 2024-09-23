import cv2
import numpy as np
import time
import rotate
try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Open a camera device for capturing
imageSize = (1280, 720)
FPS = 30
focal_length = 1760  # Provided focal length in pixels
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
hey = False
# Known real-world height of the marker (14.5 cm = 0.145 meters)
real_marker_height = 0.145

while cv2.waitKey(4) == -1:  # Wait for a key press
    # Capture frame-by-frame from the picamera
    image = cam.capture_array("main")

    # Convert the image to grayscale (ArUco detection works better in grayscale)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect ArUco markers in the grayscale image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is None:
        # Rotate the robot if no markers are detected
        rotate.rotate()
    else:
        hey = True
        # If markers are detected, estimate the pose
        # Draw detected markers on the image
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            # Get the four corners of the detected marker
            corner = corners[i][0]

            # Calculate the height of the marker in pixels (h)
            # We calculate the vertical distance between the top and bottom corners
            top_left_y = corner[0][1]
            bottom_left_y = corner[3][1]
            marker_height_in_pixels = abs(bottom_left_y - top_left_y)  # Height in pixels

            # Calculate distance Z using the formula Z = f * (H / h)
            if marker_height_in_pixels > 0:  # Prevent division by zero
                distance = focal_length * (real_marker_height / marker_height_in_pixels)

                # Display the distance on the image
                cv2.putText(image, f"Distance: {distance:.2f} m",
                            (int(corner[0][0]), int(corner[0][1]) - 10),  # Position of the text
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Print marker ID and distance in the console
                print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m")

    # Show the frame with detected markers and distance
    cv2.imshow(WIN_RF, image)

# Clean up after the loop
cam.stop()
cv2.destroyAllWindows()