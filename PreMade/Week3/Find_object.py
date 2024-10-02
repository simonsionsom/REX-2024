import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

import robot

class GridOccupancyMap(object):
    def __init__(self, low=(0, 0), high=(2, 2), res=0.05) -> None:
        self.map_area = [low, high]    # a rectangular area    
        self.map_size = np.array([high[0]-low[0], high[1]-low[1]])
        self.resolution = res

        self.n_grids = [int(s//res) for s in self.map_size]

        self.grid = np.zeros((self.n_grids[0], self.n_grids[1]), dtype=np.uint8)

        self.extent = [self.map_area[0][0], self.map_area[1][0], self.map_area[0][1], self.map_area[1][1]]

    def in_collision(self, pos):
        indices = [int((pos[i] - self.map_area[0][i]) // self.resolution) for i in range(2)]
        for i, ind in enumerate(indices):
            if ind < 0 or ind >= self.n_grids[i]:
                return 1
        
        return self.grid[indices[0], indices[1]] 

    def populate(self, n_obs=6):
        origins = np.random.uniform(
            low=self.map_area[0] + self.map_size[0]*0.2, 
            high=self.map_area[0] + self.map_size[0]*0.8, 
            size=(n_obs, 2))
        radius = np.random.uniform(low=0.1, high=0.3, size=n_obs)
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                centroid = np.array([self.map_area[0][0] + self.resolution * (i+0.5), 
                                     self.map_area[0][1] + self.resolution * (j+0.5)])
                for o, r in zip(origins, radius):
                    if np.linalg.norm(centroid - o) <= r:
                        self.grid[i, j] = 1
                        break

    def update_with_marker(self, tvec):
        pos = (tvec[0][0], tvec[0][2])  # Assuming tvec is in the form [x, y, z]
        indices = [int((pos[i] - self.map_area[0][i]) // self.resolution) for i in range(2)]
        if 0 <= indices[0] < self.n_grids[0] and 0 <= indices[1] < self.n_grids[1]:
            self.grid[indices[0], indices[1]] = 1

    def draw_map(self):
        plt.imshow(self.grid.T, cmap="Greys", origin='lower', vmin=0, vmax=1, extent=self.extent, interpolation='none')

arlo = robot.Robot()

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
intrinsic_matrix = np.asarray([ 1760, 0, 640, 
                                0, 1760, 360, 
                                0, 0, 1. ], dtype = np.float64)

intrinsic_matrix.shape = (3, 3)
map = np.zeros.array
distortion_coeffs = np.asarray([0,0,0,0,0])

def draw_aruco_objects(image, corners, ids, rvecs, tvecs):
    if ids is not None:
        outimg = cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            outimg = cv2.drawFrameAxes(outimg, intrinsic_matrix,
                                       distortion_coeffs, rvecs[i], tvecs[i], real_marker_height)
    else:
        outimg = image
    return outimg

grid_map = GridOccupancyMap(low=(0, 0), high=(5, 5), res=0.1)

while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, real_marker_height, intrinsic_matrix, distortion_coeffs)
    
    if ids is not None:
        for i in range(len(ids)):
            print("Object ID = ", ids[i], ", Distance = ", tvecs[i], ", angles = ", rvecs[i])
            grid_map.update_with_marker(tvecs[i])
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
    
    resized_image = cv2.resize(image, (320, 240))
    cv2.setWindowProperty(WIN_RF, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.imshow(WIN_RF, resized_image)

cam.stop()
cv2.destroyAllWindows()

plt.clf()
grid_map.draw_map()
plt.show()
