import cv2
import numpy as np
import time

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

import robot  # Import your robot module

# Create a robot object and initialize
arlo = robot.Robot()

# Define the speed for rotation and movement
leftSpeed = 32
rightSpeed = 32
forwardSpeed = 48  # Speed for driving forward

# Define the minimum distance threshold (in meters) to stop the robot
min_distance = 0.3  # Stop if closer than 50 cm to the marker

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

# Robot control functions (using your current rotate function)
def rotate_robot():
    print("Rotating robot...")
    # Rotate robot to the left
    arlo.go_diff(leftSpeed, rightSpeed, 1, 0)  # 1, 0 makes the robot rotate to the left
    time.sleep(0.3)
    arlo.stop()
    time.sleep(0.2)

def stop_rotation():
    print("Stopping robot rotation...")
    # Stop the robot
    arlo.stop()

def drive_forward():
    print("Driving forward...")
    # Drive robot forward
    arlo.go_diff(leftSpeed, rightSpeed, 1, 1)  # Move both wheels forward
    time.sleep(1)

def stop_robot():
    print("Stopping robot due to proximity...")
    arlo.stop()

def update_occupancy_map(center_x, center_y, distance):
    # Convert the distance and center coordinates to grid indices
    grid_x = int(center_x / grid_resolution)
    grid_y = int(center_y / grid_resolution)
    radius = int(distance / grid_resolution)

    # Mark the corresponding cells in the occupancy map as occupied
    for i in range(max(0, grid_x - radius), min(grid_size[0], grid_x + radius + 1)):
        for j in range(max(0, grid_y - radius), min(grid_size[1], grid_y + radius + 1)):
            if (i - grid_x) ** 2 + (j - grid_y) ** 2 <= radius ** 2:
                occupancy_map[i, j] = True

# Simulate the obstacle detection using a sensor
def detect_obstacle():
    # Replace this with the actual distance sensor reading
    # This is a dummy value for now. In real code, you would use arlo.read_sensor() or similar.
    obstacle_distance = arlo.read_front_ping_sensor()  # Replace with actual function call
    return obstacle_distance  # Return the distance in meters

def update_obstacle_on_map(distance, robot_position):
    # Convert the distance and robot's current heading to grid coordinates
    grid_x = int(robot_position[0] + (distance / grid_resolution))  # Adjust this for the robot's heading direction
    grid_y = int(robot_position[1])  # Assuming the obstacle is directly in front

    if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
        occupancy_map[grid_x, grid_y] = True  # Mark the obstacle cell as occupied

def visualize_occupancy_map(robot_position):
    occupancy_map_image = (occupancy_map * 255).astype(np.uint8)  # Convert to grayscale for visualization
    occupancy_map_image = cv2.cvtColor(occupancy_map_image, cv2.COLOR_GRAY2BGR)  # Convert to BGR for color drawing

    # Draw grid lines (optional for better visualization)
    for i in range(0, grid_size[0], int(1 / grid_resolution)):  # Draw vertical lines
        cv2.line(occupancy_map_image, (i, 0), (i, grid_size[1]), (255, 255, 255), 1)
    for j in range(0, grid_size[1], int(1 / grid_resolution)):  # Draw horizontal lines
        cv2.line(occupancy_map_image, (0, j), (grid_size[0], j), (255, 255, 255), 1)

    # Mark the robot's current position (red circle)
    cv2.circle(occupancy_map_image, robot_position, 5, (0, 0, 255), -1)  # Red dot for robot

    # Mark detected obstacles (green circles)
    obstacle_pos = np.where(occupancy_map == True)  # Get obstacle positions
    for i in range(len(obstacle_pos[0])):
        cv2.circle(occupancy_map_image, (obstacle_pos[0][i], obstacle_pos[1][i]), 5, (0, 255, 0), -1)  # Green dot for obstacles

    # Show the map in a window
    cv2.imshow("Occupancy Map", occupancy_map_image)


while cv2.waitKey(4) == -1: 
    image = cam.capture_array("main")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    obstacle_distance = detect_obstacle()

    print(f"corners: {corners}")
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            # Get the four corners of the detected marker
            corner = corners[i][0]
            print(f"corner number {i}: {corner}")

            # Calculate the center of the marker
            center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
            center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
            print(f"the center of the x coordinate marker: {center_x}")
            print(f"the center of the y coordinate marker: {center_y}")

            # Calculate the height of the marker in pixels (h)
            top_left_y = corner[0][1]
            bottom_left_y = corner[3][1]
            print(f"marker height top left:{top_left_y}, Marker height buttom left: {bottom_left_y}")
            marker_height_in_pixels = abs(bottom_left_y - top_left_y)  # Height in pixels

            # Calculate distance Z using the formula Z = f * (H / h)
            if marker_height_in_pixels > 0:
                distance = focal_length * (real_marker_height / marker_height_in_pixels)

                cv2.putText(image, f"Distance: {distance:.2f} m",
                            (int(corner[0][0]), int(corner[0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m, Center: ({center_x:.2f}, {center_y:.2f})")

                update_occupancy_map(center_x, center_y, distance)
                print(occupancy_map)
    else:
        None
    # Update the occupancy map with detected obstacle
    robot_position = (grid_size[0] // 2, grid_size[1] - 1)  # Example robot position
    update_obstacle_on_map(obstacle_distance, robot_position)

    # Visualize the map with markers and obstacles
    visualize_occupancy_map(robot_position)

    # Show the camera frame
    resized_image = cv2.resize(image, (639, 360))
    cv2.imshow(WIN_RF, resized_image)

cam.stop()
cv2.destroyAllWindows()