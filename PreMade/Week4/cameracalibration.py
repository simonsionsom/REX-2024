import cv2
import cv2.aruco as aruco
import numpy as np

# Load the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Create the GridBoard object
# markersX = number of markers in X direction, markersY = number in Y direction
# markerLength = 0.15 meters, markerSeparation = 0.05 meters (assuming 5 cm separation)
board = cv2.aruco.GridBoard(markersX=5, markersY=7, markerLength=0.15, markerSeparation=0.05, dictionary=aruco_dict)

# Initialize arrays to store detected corners and marker IDs
all_corners = []
all_ids = []
image_size = None

# List of calibration images
images = [...]  # Replace with your list of image file paths

# Iterate over images to detect ArUco markers
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    if image_size is None:
        image_size = gray.shape[::-1]  # Store the image size for later use

    # Detect the markers in the image
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:
        all_corners.append(corners)
        all_ids.append(ids)

# At this point, we have the detected marker corners and IDs for each image
# Assuming you already have the intrinsic matrix from earlier
# Replace these values with your actual intrinsic matrix
camera_matrix = np.array([[1760, 0, 640], [0, 1760, 360], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))  # Placeholder for distortion coefficients

# Perform the camera calibration using the ArUco markers
ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
    all_corners, all_ids, board, image_size, camera_matrix, dist_coeffs
)

# Output the distortion coefficients
print("Distortion Coefficients:\n", dist_coeffs)
