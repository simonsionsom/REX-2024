import cv2
import numpy as np
import time

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

imageSize = (1280, 720)
FPS = 30
focal_length = 1760  # Provided focal length in pixels
cam = picamera2.Picamera2()
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds

# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                queue=False)
cam.configure(picam2_config)
cam.start(show_preview=False)

time.sleep(1)  # wait for the camera to setup
# Open a window
WIN_RF = "Aruco Marker Detection"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 50, 50)

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Known real-world height of the marker (14.5 cm = 0.145 meters)
real_marker_height = 0.145

# Frame size
frame_center_x = imageSize[0] // 2
center_threshold = 350  # Threshold to consider the marker centered

# Define the grid resolution and initialize the occupancy map
grid_resolution = 0.05  # 5 cm per grid cell
grid_size = (500, 500)  # 500x500 grid = 2,5
occupancy_map = np.zeros(grid_size, dtype=bool) 

def update_occupancy_map(center_x, center_y, distance):
    # Convert the distance and center coordinates to grid indices
    grid_x = int(center_x * grid_resolution)
    true_dist = int(distance* grid_resolution)
    '''grid_y = int(center_y / grid_resolution)
    radius = int(distance / grid_resolution)'''
    print('Her er grid x: ',grid_x)
    print()
    print('Her er true_dist: ',true_dist)
    occupancy_map[grid_x,true_dist] = True


'''    # Mark the corresponding cells in the occupancy map as occupied
    for i in range(max(0, grid_x - radius), min(grid_size[0], grid_x + radius + 1)):
        for j in range(max(0, grid_y - radius), min(grid_size[1], grid_y + radius + 1)):
            if (i - grid_x) ** 2 + (j - grid_y) ** 2 <= radius ** 2:
                occupancy_map[i, j] = True'''

while cv2.waitKey(4) == -1:  # Wait for a key press
    # Capture frame-by-frame from the picamera
    image = cam.capture_array("main")

    # Convert the image to grayscale (ArUco detection works better in grayscale)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect ArUco markers in the grayscale image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print(f"corners: {corners}")
    # If markers are detected, estimate the pose
    if ids is not None:
        # Draw detected markers on the image
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            # Get the four corners of the detected marker
            corner = corners[i][0]
            print(f"Box number {i}: {corner}")

            # Calculate the center of the marker
            center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
            print()
            print(f'Here is center x: {center_x}, and here is are the corners: {corner}')
            center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
            '''print(f"the center of the x coordinate marker: {center_x}")
            print(f"the center of the y coordinate marker: {center_y}")'''
            # Calculate the height of the marker in pixels (h)
            top_left_y = corner[0][1]
            bottom_left_y = corner[3][1]
            print(f"marker height top left:{top_left_y}, Marker height buttom left: {bottom_left_y}")
            marker_height_in_pixels = abs(bottom_left_y - top_left_y)  # Height in pixels
            # Calculate distance Z using the formula Z = f * (H / h)
            if marker_height_in_pixels > 0:  # Prevent division by zero
                distance = focal_length * (real_marker_height / marker_height_in_pixels)

                '''# Display the distance on the image
                cv2.putText(image, f"Distance: {distance:.2f} m",
                            (int(corner[0][0]), int(corner[0][1]) - 10),  # Position of the text
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)'''

                # Print marker ID, center, and distance in the console
                print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m, Center: ({center_x:.2f}, {center_y:.2f})")

                update_occupancy_map(center_x, center_y, distance)
                
                # Check if the marker is centered within the threshold
    else:
        # No markers detected, rotate the robot
        None
    #cv2.resizeWindow(WIN_RF, 500, 500)

    resized_image = cv2.resize(image, (639,360))  # Resize to a smaller size
  
    # Show the frame with detected markers and distance
    cv2.imshow(WIN_RF, resized_image)
    # Show the frame with detected markers and distance

    # Visualize the occupancy map
    #occupancy_map_image = (occupancy_map * 255).astype(np.uint8)
    #cv2.imshow("Occupancy Map", occupancy_map_image)

# Clean up after the loop
print(occupancy_map)
cam.stop()
cv2.destroyAllWindows()
