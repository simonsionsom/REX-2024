# import cv2
# import numpy as np
# import time
# import random
# import rotate
# try:
#     import picamera2
#     print("Camera.py: Using picamera2 module")
# except ImportError:
#     print("Camera.py: picamera2 module not available")
#     exit(-1)

# import robot  # Import your robot module

# # Create a robot object and initialize
# arlo = robot.Robot()

# # Define the speed for rotation and movement
# leftSpeed = 32
# rightSpeed = 32
# forwardSpeed = 48  # Speed for driving forward

# # Define the minimum distance threshold (in meters) to stop the robot
# min_distance = 0.3  # Stop if closer than 30 cm to the marker

# # Open a camera device for capturing
# imageSize = (1280, 720)
# FPS = 30
# focal_length = 1760  # Provided focal length in pixels
# cam = picamera2.Picamera2()
# frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds

# # Change configuration to set resolution, framerate
# picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
#                                                 controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
#                                                 queue=False)
# cam.configure(picam2_config)
# cam.start(show_preview=False)

# time.sleep(1)  # wait for the camera to setup

# # Open a window
# WIN_RF = "Aruco Marker Detection"
# cv2.namedWindow(WIN_RF)
# cv2.moveWindow(WIN_RF, 100, 100)

# # Load the ArUco dictionary and create detector parameters
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
# parameters = cv2.aruco.DetectorParameters()

# # Known real-world height of the marker (14.5 cm = 0.145 meters)
# real_marker_height = 0.145

# # Frame size
# frame_center_x = imageSize[0] // 2
# center_threshold = 350  # Threshold to consider the marker centered

# def rotate_robot():
#     print("Rotating robot...")
#     # Rotate robot to the left
#     arlo.go_diff(leftSpeed, rightSpeed, 1, 0)  # 1, 0 makes the robot rotate to the left
#     time.sleep(0.3)
#     arlo.stop()
#     time.sleep(0.2)

# def stop_rotation():
#     print("Stopping robot rotation...")
#     # Stop the robot
#     arlo.stop()

# def drive_forward():
#     print("Driving forward...")
#     # Drive robot forward
#     arlo.go_diff(leftSpeed, rightSpeed, 1, 1)  # Move both wheels forward
#     time.sleep(1)

# def stop_robot():
#     print("Stopping robot due to proximity...")
#     arlo.stop()

# # Define a simple 2D point class
# class Point:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y

# # Define a Node class for the tree
# class Node:
#     def __init__(self, point, parent=None):
#         self.point = point
#         self.parent = parent

# # Function to calculate the Euclidean distance between two points
# def distance(p1, p2):
#     return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

# # RRT algorithm class
# class RRT:
#     def __init__(self, start, goal, obstacle_list, map_size):
#         self.start = Node(start)
#         self.goal = Node(goal)
#         self.obstacle_list = obstacle_list
#         self.map_size = map_size
#         self.node_list = [self.start]

#     def get_random_point(self):
#         return Point(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

#     def get_nearest_node(self, random_point):
#         return min(self.node_list, key=lambda node: distance(node.point, random_point))

#     def is_collision_free(self, point1, point2):
#         for (ox, oy, size) in self.obstacle_list:
#             if distance(point1, Point(ox, oy)) <= size or distance(point2, Point(ox, oy)) <= size:
#                 return False
#         return True

#     def extend(self, nearest_node, random_point):
#         direction = np.array([random_point.x - nearest_node.point.x, random_point.y - nearest_node.point.y])
#         direction = direction / np.linalg.norm(direction)  # Normalize direction
#         new_point = Point(nearest_node.point.x + direction[0], nearest_node.point.y + direction[1])
#         if self.is_collision_free(nearest_node.point, new_point):
#             new_node = Node(new_point, nearest_node)
#             self.node_list.append(new_node)
#             return new_node
#         return None

#     def plan(self, max_iterations=1000):
#         for _ in range(max_iterations):
#             random_point = self.get_random_point()
#             nearest_node = self.get_nearest_node(random_point)
#             new_node = self.extend(nearest_node, random_point)
#             if new_node and distance(new_node.point, self.goal.point) < 1.0:
#                 self.goal.parent = new_node
#                 self.node_list.append(self.goal)
#                 return self.extract_path()
#         return None  # No path found

#     def extract_path(self):
#         path = []
#         node = self.goal
#         while node.parent is not None:
#             path.append(node.point)
#             node = node.parent
#         path.append(self.start.point)
#         return path[::-1]

# # Function to move the robot along the planned path
# def follow_path(path):
#     for point in path:
#         drive_forward(point.x, point.y)
#         time.sleep(1)

# # Example robot move_to function (to be replaced with actual robot control)
# def move_robot_to_position(robot, x, y):
#     print(f"Moving robot to ({x}, {y})")
#     # Add your robot's movement control code here
#     robot.go_diff(leftSpeed, rightSpeed, 1, 1)  # Example move forward
#     time.sleep(1)
#     robot.stop()

# while cv2.waitKey(4) == -1:  # Wait for a key press
#     # Capture frame-by-frame from the picamera
#     image = cam.capture_array("main")

#     # Convert the image to grayscale (ArUco detection works better in grayscale)
#     gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

#     # Detect ArUco markers in the grayscale image
#     corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

#     # If markers are detected, estimate the pose
#     if ids is not None:
#         # Draw detected markers on the image
#         cv2.aruco.drawDetectedMarkers(image, corners, ids)
#         for i in range(len(ids)):
#             # Get the four corners of the detected marker
#             corner = corners[i][0]

#             # Calculate the center of the marker
#             center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
#             center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4

#             # Calculate the height of the marker in pixels (h)
#             top_left_y = corner[0][1]
#             bottom_left_y = corner[3][1]
#             marker_height_in_pixels = abs(bottom_left_y - top_left_y)  # Height in pixels

#             # Calculate distance Z using the formula Z = f * (H / h)
#             if marker_height_in_pixels > 0:  # Prevent division by zero
#                 distance = focal_length * (real_marker_height / marker_height_in_pixels)

#                 # Display the distance on the image
#                 cv2.putText(image, f"Distance: {distance:.2f} m",
#                             (int(corner[0][0]), int(corner[0][1]) - 10),  # Position of the text
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

#                 # Print marker ID, center, and distance in the console
#                 print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m, Center: ({center_x:.2f}, {center_y:.2f})")

#                 # RRT path planning
#                 start_position = Point(0, 0)  # This should be the robot's current position
#                 goal_position = Point(center_x, distance)  # Goal is the detected marker's position

#                 # Example obstacle list (x, y, radius)
#                 obstacle_list = [(5, 5, 1.5), (3, 6, 2), (7, 8, 2)]
#                 map_size = (15, 15)  # Define the map size

#                 rrt = RRT(start_position, goal_position, obstacle_list, map_size)
#                 path = rrt.plan()

#                 if path:
#                     follow_path(arlo, path)
#                 else:
#                     print("No path found")

#     else:
#         # No markers detected, rotate the robot
#         rotate_robot()

#     # Show the frame with detected markers and distance
#     cv2.imshow(WIN_RF, image)

# # Clean up after the loop
# cam.stop()
# cv2.destroyAllWindows()
import cv2
import numpy as np
import time
import random

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
min_distance = 0.3  # Stop if closer than 30 cm to the marker

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
cv2.moveWindow(WIN_RF, 100, 100)

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Known real-world height of the marker (14.5 cm = 0.145 meters)
real_marker_height = 0.145

# Frame size
frame_center_x = imageSize[0] // 2
center_threshold = 350  # Threshold to consider the marker centered

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

# Define a simple 2D point class
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Define a Node class for the tree
class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent

# Function to calculate the Euclidean distance between two points
def euclidean_distance(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

# RRT algorithm class
class RRT:
    def __init__(self, start, goal, obstacle_list, map_size):
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacle_list = obstacle_list
        self.map_size = map_size
        self.node_list = [self.start]

    def get_random_point(self):
        return Point(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

    def get_nearest_node(self, random_point):
        return min(self.node_list, key=lambda node: euclidean_distance(node.point, random_point))

    def is_collision_free(self, point1, point2):
        for (ox, oy, size) in self.obstacle_list:
            if euclidean_distance(point1, Point(ox, oy)) <= size or euclidean_distance(point2, Point(ox, oy)) <= size:
                return False
        return True

    def extend(self, nearest_node, random_point):
        direction = np.array([random_point.x - nearest_node.point.x, random_point.y - nearest_node.point.y])
        direction = direction / np.linalg.norm(direction)  # Normalize direction
        new_point = Point(nearest_node.point.x + direction[0], nearest_node.point.y + direction[1])
        if self.is_collision_free(nearest_node.point, new_point):
            new_node = Node(new_point, nearest_node)
            self.node_list.append(new_node)
            return new_node
        return None

    def plan(self, max_iterations=1000):
        for _ in range(max_iterations):
            random_point = self.get_random_point()
            nearest_node = self.get_nearest_node(random_point)
            new_node = self.extend(nearest_node, random_point)
            if new_node and euclidean_distance(new_node.point, self.goal.point) < 1.0:
                self.goal.parent = new_node
                self.node_list.append(self.goal)
                return self.extract_path()
        return None  # No path found

    def extract_path(self):
        path = []
        node = self.goal
        while node.parent is not None:
            path.append(node.point)
            node = node.parent
        path.append(self.start.point)
        return path[::-1]

# Function to move the robot along the planned path
def follow_path(robot, path):
    for point in path:
        move_robot_to_position(robot, point.x, point.y)
        time.sleep(1)

# Example robot move_to function (to be replaced with actual robot control)
def move_robot_to_position(robot, x, y):
    print(f"Moving robot to ({x}, {y})")
    # Add your robot's movement control code here
    robot.go_diff(leftSpeed, rightSpeed, 1, 1)  # Example move forward
    time.sleep(1)
    robot.stop()

while cv2.waitKey(4) == -1:  # Wait for a key press
    # Capture frame-by-frame from the picamera
    image = cam.capture_array("main")

    # Convert the image to grayscale (ArUco detection works better in grayscale)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect ArUco markers in the grayscale image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected, estimate the pose
    if ids is not None:
        # Draw detected markers on the image
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            # Get the four corners of the detected marker
            corner = corners[i][0]

            # Calculate the center of the marker
            center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
            center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4

            # Calculate the height of the marker in pixels (h)
            top_left_y = corner[0][1]
            bottom_left_y = corner[3][1]
            marker_height_in_pixels = abs(bottom_left_y - top_left_y)  # Height in pixels

            # Calculate distance Z using the formula Z = f * (H / h)
            if marker_height_in_pixels > 0:  # Prevent division by zero
                distance_to_marker = focal_length * (real_marker_height / marker_height_in_pixels)

                # Display the distance on the image
                cv2.putText(image, f"Distance: {distance_to_marker:.2f} m",
                            (int(corner[0][0]), int(corner[0][1]) - 10),  # Position of the text
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Print marker ID, center, and distance in the console
                print(f"Marker ID: {ids[i][0]}, Distance: {distance_to_marker:.2f} m, Center: ({center_x:.2f}, {center_y:.2f})")

                # RRT path planning
                start_position = Point(0, 0)  # This should be the robot's current position
                goal_position = Point(center_x, distance_to_marker)  # Goal is the detected marker's position

                # Example obstacle list (x, y, radius)
                obstacle_list = [(5, 5, 1.5), (3, 6, 2), (7, 8, 2)]
                map_size = (15, 15)  # Define the map size

                rrt = RRT(start_position, goal_position, obstacle_list, map_size)
                path = rrt.plan()

                if path:
                    follow_path(arlo, path)
                else:
                    print("No path found")

    else:
        # No markers detected, rotate the robot
        rotate_robot()

    # Show the frame with detected markers and distance
    cv2.imshow(WIN_RF, image)

# Clean up after the loop
cam.stop()
cv2.destroyAllWindows()
