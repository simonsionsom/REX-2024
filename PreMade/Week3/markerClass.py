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

class Localizer:
    def __init__(self, low=(0, 0), high=(5, 5), res=0.05) -> None:
        # Initialize robot and camera
        self.arlo = robot.Robot()

        # Camera configuration
        self.imageSize = (1280, 720)
        self.FPS = 30
        self.focal_length = 1760
        self.cam = picamera2.Picamera2()
        self.frame_duration_limit = int(1 / self.FPS * 1000000)

        picam2_config = self.cam.create_video_configuration({"size": self.imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (self.frame_duration_limit, self.frame_duration_limit)},
                                                            queue=False)
        self.cam.configure(picam2_config)
        self.cam.start(show_preview=False)

        # ArUco marker settings
        self.WIN_RF = "Aruco Marker Detection"
        cv2.namedWindow(self.WIN_RF)
        cv2.moveWindow(self.WIN_RF, 100, 100)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Marker detection parameters
        self.real_marker_height = 0.15
        self.frame_center_x = self.imageSize[0] // 2
        self.center_threshold = 350

        # Intrinsic matrix for camera
        self.intrinsic_matrix = np.asarray([self.focal_length, 0, self.imageSize[0] / 2.0, 
                                            0, self.focal_length, self.imageSize[1] / 2.0, 
                                            0, 0, 1.], dtype=np.float64)
        self.intrinsic_matrix.shape = (3, 3)

        # Distortion coefficients (no distortion in this case)
        self.distortion_coeffs = np.asarray([0, 0, 0, 0, 0])

        # Grid map setup

        self.map_area = [low, high]    #a rectangular area    
        self.map_size = np.array([high[0]-low[0], high[1]-low[1]])
        self.resolution = res
        self.gridSize = 5

        # Number of grids based on resolution
        self.n_grids = [ int(s//res) for s in self.map_size]

        # Initialize an empty grid
        self.grid = np.zeros((self.n_grids[0], self.n_grids[1]), dtype=np.uint8)

        # Plotting extent for visualization
        self.extent = [self.map_area[0][0], self.map_area[1][0], self.map_area[0][1], self.map_area[1][1]]

    def populate(self, boxes):
        ahhhh = []
        radius = 6
        midP = (int(self.n_grids[0] / 2), 0)
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                centroid = np.array([0.5 + i, 0.5 + j])

                for o, f in boxes:
                    if np.linalg.norm(centroid - (o + midP)) <= radius:
                        print('We did it')
                        self.grid[i, j] = 1
                        ahhhh.append(((i, j), f))
                        break
        return ahhhh

    def find_lengths(self, corners, ids):
        distances = []
        for i in range(0, len(corners)):
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.real_marker_height,
                                                                          self.intrinsic_matrix, self.distortion_coeffs)
            dist = np.array((tvecs.T[0][0][0] * 100, tvecs.T[2][0][0] * 100))
            distances.append((dist / self.gridSize, ids[i]))
            print(dist)
        print(distances)
        return distances

    def run(self):
        while True:
            image = self.cam.capture_array("main")
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            # Calculate distances based on marker detection
            distances = self.find_lengths(corners, ids)

            # Populate grid based on distances
            print(self.populate(distances))

            # Save grid state to file
            np.save('map.npy', self.grid)

            # Break after one iteration for testing purposes
            break

        # Clean up
        cv2.destroyAllWindows()

# Usage
localizer = Localizer()
localizer.run()
