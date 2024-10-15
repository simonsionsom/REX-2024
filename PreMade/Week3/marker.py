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

real_marker_height = 0.15

frame_center_x = imageSize[0] // 2
center_threshold = 350
intrinsic_matrix = np.asarray([focal_length, 0, imageSize[0] / 2.0, 
                                0, focal_length, imageSize[1] / 2.0, 
                                0, 0, 1. ], dtype = np.float64)

intrinsic_matrix.shape = (3, 3)

low = (0,0)
high = (5,5)

map_area = [low, high]    #a rectangular area    
map_size = np.array([high[0]-low[0], high[1]-low[1]])
resolution = 0.05
gridSize= 5

n_grids = [ int(s//resolution) for s in map_size]
print(n_grids,'\n')
grid = np.zeros((n_grids[0], n_grids[1]), dtype=np.uint8)
midP = int(n_grids[0]/2)


extent = [map_area[0][0], map_area[1][0], map_area[0][1], map_area[1][1]]



# distortion_coeffs = np.asarray([3.37113443e+00, -5.84490229e+01,
#        -9.99698589e-02, -2.84566227e-02, 1.18763855e+03], dtype = np.float64)
distortion_coeffs = np.asarray([0,0,0,0,0])

def populate(boxes):
    radius=5

    for i in range(n_grids[0]):
        for j in range(n_grids[1]):
            centroid = np.array([0.5+i, 
                                     0.5+j])
            for o in boxes:
                    o[0] = o[0]+midP
                    print(o)
                    if np.linalg.norm(centroid - o) <= radius:   
                        #if np.linalg.norm(int(o[0])*resolution-high[1]) <= high[1]:
                            #print(f'Her er den nye bokses x-koordinat: {int(o[0])}')
                            #o[0]=midP+int(o[0])
                            print('We did it')
                            grid[i, j] = 1
                            break
                        #else: 
                            #print(f'Skipped den her box: {int(o)}\n med en x-kordinat på {int(o[0])}')
                            #continue

def find_Lengths(corners):
    distances = []
    for i in corners:
        rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(i, real_marker_height, intrinsic_matrix, distortion_coeffs)
        dist = np.array((tvecs.T[0][0][0]*100,tvecs.T[2][0][0]*100))
        #print(f'Her er tvec{tvecs.T},\n Her er distancen så ing {dist}')
        distances.append(dist/gridSize)
    #print(distances)
    return distances


while True:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print('\nHer er ids',ids)
    #print('\nHer er corners.shape',corners)
    distances = find_Lengths(corners)
    #print('\nHer er distances',distances)
    
    populate(distances)
    np.save('map.npy',grid)
    # Use OpenCV to display the grid map instead of plt
    #draw_map(grid)
    break
    

# Clean up when done
#cam.stop()
cv2.destroyAllWindows()
