# import cv2
# import cv2.aruco as aruco
# import numpy as np

# # Load the ArUco dictionary
# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# # Create the GridBoard object
# # markersX = number of markers in X direction, markersY = number in Y direction
# # markerLength = 0.15 meters, markerSeparation = 0.05 meters (assuming 5 cm separation)
# board = cv2.aruco.GridBoard((5, 7), markerLength=0.15, markerSeparation=0.05, dictionary=aruco_dict)

# # Initialize arrays to store detected corners and marker IDs
# all_corners = []
# all_ids = []
# image_size = None

# # List of calibration images
# images = [...]  # Replace with your list of image file paths

# # Iterate over images to detect ArUco markers
# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
#     if image_size is None:
#         image_size = gray.shape[::-1]  # Store the image size for later use

#     # Detect the markers in the image
#     corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

#     if ids is not None:
#         all_corners.append(corners)
#         all_ids.append(ids)

# # At this point, we have the detected marker corners and IDs for each image
# # Assuming you already have the intrinsic matrix from earlier
# # Replace these values with your actual intrinsic matrix
# camera_matrix = np.array([[1760, 0, 640], [0, 1760, 360], [0, 0, 1]])
# dist_coeffs = np.zeros((5, 1))  # Placeholder for distortion coefficients

# # Perform the camera calibration using the ArUco markers
# ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
#     all_corners, all_ids, board, image_size, camera_matrix, dist_coeffs
# )

# # Output the distortion coefficients
# print("Distortion Coefficients:\n", dist_coeffs)
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from pprint import pprint

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Load the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Correct initialization of GridBoard
board = aruco.GridBoard((5, 7), 0.15, 0.05, aruco_dict)

# Initialize arrays to store detected corners, marker IDs, and counters
all_corners = []
all_ids = []
counter = []
image_size = None

# Configure the picamera2
imageSize = (1280, 720)  # Set the resolution
FPS = 30  # Set the frames per second
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000)  # Microseconds

# Set up the camera configuration
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                queue=False)
cam.configure(picam2_config)
cam.start(show_preview=False)

pprint(cam.camera_configuration())  # Print the camera configuration in use

# Wait for the camera to set up
time.sleep(1)

# Open a window for display
WIN_RF = "Camera Calibration with ArUco"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# Capture frames from the camera and detect ArUco markers
capture_count = 0  # Count the number of frames captured for calibration

while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")  # Capture image from camera
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

    if image_size is None:
        image_size = gray.shape[::-1]  # Store the image size for calibration

    # Detect the ArUco markers in the captured image
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:
        aruco.drawDetectedMarkers(image, corners, ids)  # Draw detected markers on the image
        
        # Ensure corners are float32
        corners = [np.array(c, dtype=np.float32) for c in corners]
        
        # Flatten the list of corners and append to the global list
        all_corners.extend(corners)
        
        # Convert ids to a numpy array of type int32 and flatten
        ids = np.array(ids, dtype=np.int32)
        all_ids.extend(ids)
        
        # Append the number of markers detected in this frame
        counter.append(len(ids))
        
        capture_count += 1
        print(f"Captured frame {capture_count} for calibration")

    # Show the captured frame with detected markers
    cv2.imshow(WIN_RF, image)

    # Break the loop when enough images have been captured
    if capture_count >= 10:  # Capture 10 frames for calibration
        break

# Stop the camera and close the display window
cam.stop()
cv2.destroyAllWindows()

# Initialize your intrinsic matrix (you already have values for focal lengths and principal point)
camera_matrix = np.array([[1760, 0, 640], [0, 1760, 360], [0, 0, 1]])

# Initialize distortion coefficients as a 2D array with shape (5, 1)
dist_coeffs = np.zeros((5, 1))  # Placeholder for distortion coefficients (2D array)

# Convert ids to numpy array again before passing to calibrateCameraAruco
all_ids = np.array(all_ids, dtype=np.int32)

# Perform the camera calibration using the ArUco markers
ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
    all_corners, all_ids, counter, board, image_size, camera_matrix, dist_coeffs
)

# Output the updated camera matrix and distortion coefficients
print("Calibration successful!")
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)

