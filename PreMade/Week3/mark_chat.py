import cv2
import time
import numpy as np
import matplotlib.pyplot as plt

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

import robot

np.set_printoptions(threshold=np.inf)

arlo = robot.Robot()

# Camera and marker setup
imageSize = (1280, 720)
FPS = 30
focal_length = 1760
cam = picamera2.Picamera2()
frame_duration_limit = int(1 / FPS * 1000000)

# Camera configuration
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                queue=False)
cam.configure(picam2_config)
cam.start(show_preview=False)

time.sleep(1)

WIN_RF = "Aruco Marker Detection"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# ArUco marker settings
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Robot and grid parameters
real_marker_height = 0.15
frame_center_x = imageSize[0] // 2
center_threshold = 350
intrinsic_matrix = np.array([[focal_length, 0, imageSize[0] / 2.0],
                             [0, focal_length, imageSize[1] / 2.0],
                             [0, 0, 1.0]], dtype=np.float64)
distortion_coeffs = np.zeros(5)

# Map and grid settings
low = (0, 0)
high = (2, 2)
map_area = [low, high]  # A rectangular area
map_size = np.array([high[0] - low[0], high[1] - low[1]])
resolution = 0.025
n_grids = [int(s // resolution) for s in map_size]
grid = np.zeros((n_grids[0], n_grids[1]), dtype=np.uint8)
extent = [map_area[0][0], map_area[1][0], map_area[0][1], map_area[1][1]]

# Function definitions
def populate(boxes):
    radius = 0.25
    for i in range(n_grids[0]):
        for j in range(n_grids[1]):
            centroid = np.array([map_area[0][0] + resolution * (i + 0.5), 
                                 map_area[0][1] + resolution * (j + 0.5)])
            for o, k in boxes:
                # Ensure o and k are within grid bounds before accessing grid[o, k]
                if 0 <= o < n_grids[0] and 0 <= k < n_grids[1]:
                    if np.linalg.norm(centroid - np.array([o, k]) * resolution) <= radius:
                        grid[i, j] = 1
                        break

def find_lengths(corners):
    distances = []
    if corners is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, real_marker_height, intrinsic_matrix, distortion_coeffs)
        for tvec in tvecs:
            dist = (tvec[0][0] / resolution, tvec[0][2] / resolution)
            distances.append((int(dist[0]), int(dist[1])))
    return distances

def draw_map():
    display_grid = (grid * 255).astype(np.uint8)
    resized_grid = cv2.resize(display_grid, (480, 360), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Grid Map", resized_grid)

# Main loop
while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners:
        distances = find_lengths(corners)
        populate(distances)
    draw_map()

# Clean up
cam.stop()
cv2.destroyAllWindows()
