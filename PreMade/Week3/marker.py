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
high = (2,2)

map_area = [low, high]    #a rectangular area    
map_size = np.array([high[0]-low[0], high[1]-low[1]])
resolution = 0.025

n_grids = [ int(s//resolution) for s in map_size]

grid = np.zeros((n_grids[0], n_grids[1]), dtype=np.uint8)

extent = [map_area[0][0], map_area[1][0], map_area[0][1], map_area[1][1]]

distances = []

# distortion_coeffs = np.asarray([3.37113443e+00, -5.84490229e+01,
#        -9.99698589e-02, -2.84566227e-02, 1.18763855e+03], dtype = np.float64)
distortion_coeffs = np.asarray([0,0,0,0,0])
'''def draw_aruco_objects(image, corners, ids, rvecs, tvecs,self):
    """Draws detected ArUco markers and their orientations on the image."""
    if ids is not None:
        # Draw the detected markers
        outimg = cv2.aruco.drawDetectedMarkers(image, corners, ids)
        # Draw the axis for each detected marker
        for i in range(len(ids)):
            outimg = cv2.drawFrameAxes(outimg, intrinsic_matrix,
                                       distortion_coeffs, rvecs[i], tvecs[i], real_marker_height)
    else:
        outimg = image
    return outimg'''

def populate(boxes):
    radius=0.25
    for i in range(n_grids[0]):
        for j in range(n_grids[1]):
            centroid = np.array([map_area[0][0] + resolution * (i+0.5), 
                                     map_area[0][1] + resolution * (j+0.5)])
            for o in boxes:
                    if np.linalg.norm(centroid - o) <= radius:
                        grid[i, j] = 1
                        break

def find_Lengths(corners):
    
    for x in corners:
        rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, real_marker_height, intrinsic_matrix, distortion_coeffs)
        dist = np.array([tvecs.T[0][0], tvecs.T[2][0]]) / resolution
        
        distances.append(dist)
        print(dist)


def draw_map():
    display_grid = (grid * 255).astype(np.uint8)
    
    # Resize the grid to the same size as the image for visualization
    resized_grid = cv2.resize(display_grid, (480,360), interpolation=cv2.INTER_NEAREST)
    
    # Show the grid using OpenCV's imshow
    cv2.imshow("Grid Map", resized_grid)

'''while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    distances = find_Lengths(corners)
    populate(distances)
    plt.clf()
    draw_map()
    plt.pause(0.001)
    plt.show()
    '''
# if ids is not None:
#        for i in range(len(ids)):
#            print("Object ID = ", ids[i], ", Distance = ", tvecs[i], ", angles = ", rvecs[i])
#        cv2.aruco.drawDetectedMarkers(image, corners, ids)
#    print(tvecs.T)
#    z = tvecs.T[2][0]*100
#    x = tvecs.T[0][0]*100
#    print(f'Here is z:{z}\n And here is x:{x}')
#    map = np.zeros((40,40))
#    for i in range(0,3):
#        for j in range(0,3):
#            map[int(z/5)-i,int(x/5)-j]=1
'''
    resized_image = cv2.resize(image, (320, 240))
    cv2.imshow(WIN_RF, resized_image)
print(map)
cam.stop()
cv2.destroyAllWindows()
'''

while cv2.waitKey(4) == -1:
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    find_Lengths(corners)
    
    # Find lengths and update grid
    #print(distances)
    populate(distances)
    
    # Use OpenCV to display the grid map instead of plt
    draw_map()

    # Display the resized image from the camera
    resized_image = cv2.resize(image, (320, 240))

    cv2.imshow(WIN_RF, resized_image)

# Clean up when done
cam.stop()
cv2.destroyAllWindows()