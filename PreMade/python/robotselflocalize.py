import cv2
import particle
import camera
import numpy as np
import time
from timeit import default_timer as timer
import sys
import math


# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True  # Whether or not we are running on the Arlo robot


def compute_weight(measured_distance, predicted_distance, sigma):
    """Computes the weight based on a Gaussian distribution."""
    diff = measured_distance - predicted_distance
    weight = (1.0 / (math.sqrt(2 * math.pi) * sigma)) * math.exp(-(diff ** 2) / (2 * sigma ** 2))
    return weight

def compute_angle_difference(angle1, angle2):
    """Compute the difference between two angles, ensuring it wraps within [-pi, pi]."""
    diff = angle1 - angle2
    while diff > np.pi:
        diff -= 2 * np.pi
    while diff < -np.pi:
        diff += 2 * np.pi
    return -diff

def resample_particles(particles):
    """Resample particles based on their weights using systematic resampling."""
    num_particles = len(particles)
    
    # Step 1: Normalize the weights
    weights = np.array([p.getWeight() for p in particles])
    weights /= np.sum(weights)  # Normalize weights so they sum to 1

    # Step 2: Compute the cumulative sum of the weights
    cumulative_sum = np.cumsum(weights)

    # Step 3: Generate random starting point between 0 and 1/num_particles
    r = np.random.uniform(0, 1.0 / num_particles)

    # Step 4: Systematic resampling
    new_particles = []
    i = 0
    for j in range(num_particles):
        u = r + j * (1.0 / num_particles)
        while u > cumulative_sum[i]:
            i += 1

        # Manually copy the particle's attributes
        selected_particle = particles[i]
        new_particle = particle.Particle(
            selected_particle.getX(),
            selected_particle.getY(),
            selected_particle.getTheta(),
            # 1.0 / num_particles  # Reset weight to uniform
            selected_particle.getWeight()
        )
        new_particles.append(new_particle)

    return new_particles


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


# if isRunningOnArlo():
#     # XXX: You need to change this path to point to where your robot.py file is located
#     #sys.path.append("../../../../Arlo/python")
#     sys.path.append("../Week3")


try:
    import robot
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False




# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [3, 4]
landmarks = {
    3: (0.0, 0.0),  # Coordinates for landmark 1
    4: (100.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks





def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    # a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    # b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
    #                              ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    # cv2.circle(world, a, 5, CMAGENTA, 2)
    # cv2.line(world, a, b, CMAGENTA, 2)
    a_robot = (int(robot_pose.getX()) + offsetX, ymax - (int(robot_pose.getY()) + offsetY))
    b_robot = (int(robot_pose.getX() + 15.0 * np.cos(robot_pose.getTheta())) + offsetX, 
           ymax - (int(robot_pose.getY() + 15.0 * np.sin(robot_pose.getTheta())) + offsetY))
    cv2.circle(world, a_robot, 5, CYELLOW, 2)  # Draw robot with a different color (yellow, for example)
    cv2.line(world, a_robot, b_robot, CYELLOW, 2)




def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles


# Main program #
try:
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1, cv2.WINDOW_NORMAL)
        cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)


    # Initialize particles
    num_particles = 1000
    particles = initialize_particles(num_particles)

    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec

    # Initialize the robot (XXX: You do this)
    robot_pose = particle.Particle(100, 240, 0, 0)

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    draw_world(est_pose, particles, world)

    print("Opening and initializing camera")
    if isRunningOnArlo():
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=False)

    while True:

        
        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
            elif action == ord('x'): # Backwards
                velocity -= 4.0
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
            elif action == ord('d'): # Right
                angular_velocity -= 0.2
        
        dt = 0.1
        for p in particles:
            delta_theta = angular_velocity * dt
            delta_x = velocity * dt * np.cos(p.getTheta())
            delta_y = velocity * dt * np.sin(p.getTheta())
            # delta_x = velocity * dt * np.cos(est_pose.getTheta())
            # delta_y = velocity * dt * np.sin(est_pose.getTheta())
            particle.move_particle(p, delta_x, delta_y, delta_theta)

        
        delta_theta_robot = angular_velocity * dt
        delta_x_robot = velocity * dt * np.cos(robot_pose.getTheta())
        delta_y_robot = velocity * dt * np.sin(robot_pose.getTheta())

        # Update the robot's pose
        particle.move_particle(robot_pose, delta_x_robot, delta_y_robot, delta_theta_robot)
        
            
            


        
        # Use motor controls to update particles
        # XXX: Make the robot drive
        # XXX: You do this


        # Fetch next frame
        colour = cam.get_next_frame()
        robot_measured_distances = {
            3: 200.0,  # Example distance to landmark 1
            4: 300.0   # Example distance to landmark 2
        }
        sigma = 10
        sigma_angle = 0.15
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(objectIDs, type(None)):

            detected_objects = {}
            # List detected objects
            for i in range(len(objectIDs)):
                print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
                # XXX: Do something for each detected object - remember, the same ID may appear several times
            # List detected objects
            for i in range(len(objectIDs)):
                obj_id = objectIDs[i]
                obj_dist = dists[i]
                obj_angle = angles[i]
        
                print(f"Object ID = {obj_id}, Distance = {obj_dist}, Angle = {obj_angle}")
        
                # Check if this object ID has already been detected
                if obj_id not in detected_objects:
                    # If it's the first detection of this ID, store the distance and angle
                    detected_objects[obj_id] = {"distance": obj_dist, "angle": obj_angle}
                else:
                    # If the object was detected before, keep the shortest distance
                    if obj_dist < detected_objects[obj_id]["distance"]:
                        detected_objects[obj_id] = {"distance": obj_dist, "angle": obj_angle}
            # Now process detected_objects and take action
            particle_weights = {}
            for obj_id, info in detected_objects.items():
                shortest_distance = info["distance"]
                corresponding_angle = info["angle"]
        
                print(f"Object ID {obj_id} - Shortest Distance: {shortest_distance}, Corresponding Angle: {corresponding_angle}")
            # Compute particle weights
            # XXX: You do this
            # Compute particle weights based on measured distances
            # Har ikke fået testet på 2 landmarks med denne her kode. Fjern 314 og 335 for at ændre koden tilbage hvis ikke det her virker
            # og udkommenter 313 og 334 :)
            for p in particles:
                particle_weights[p] = 1.0
                # particle_weights[p] = 0.0  # Initialize with neutral weight

                for obj_id, info in detected_objects.items():
                    # Get measured distance and corresponding angle
                    measured_distance = info["distance"]
                    measured_angle = info["angle"]

                    # Predicted distance from particle to landmark
                    landmark_x, landmark_y = landmarks[obj_id]
                    particle_x, particle_y, particle_theta = p.getX(), p.getY(), p.getTheta()

                    predicted_distance = np.sqrt((landmark_x - particle_x) ** 2 + (landmark_y - particle_y) ** 2)
                    predicted_angle = np.arctan2(landmark_y - particle_y, landmark_x - particle_x)

                    angle_difference = compute_angle_difference(predicted_angle, particle_theta)
                    # Compute weight based on how close the predicted distance is to the measured distance
                    weight_distance = compute_weight(measured_distance, predicted_distance, sigma)
                    weight_angle = compute_weight(measured_angle, angle_difference, sigma_angle)

                    combined_weight = weight_distance * weight_angle
                    particle_weights[p] *= weight_distance * weight_angle
                    # particle_weights[p] += combined_weight

            # Normalize the particle weights so they sum to 1
            total_weight = sum(particle_weights.values())
            print(f"Total weight = {total_weight}")
            if total_weight > 0:
                for p in particles:
                    p.setWeight(particle_weights[p] / total_weight)
            else:
                # If no valid observations, reset to uniform weights
                for p in particles:
                    p.setWeight(1.0 / num_particles)
            # Resampling
            # XXX: You do this
            print(f"Number of particles before resampling: {len(particles)}")
            particles = resample_particles(particles)
            print(f"Number of particles after resampling: {len(particles)}")
            particle.add_uncertainty(particles, sigma, sigma_angle)

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)

    
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)
    
            # Show frame
            cv2.imshow(WIN_RF1, colour)
            cv2.resizeWindow(WIN_RF1, 640, 360)

            # Show world
            cv2.imshow(WIN_World, world)
    
  
finally: 
    # Make sure to clean up even if an exception occurred
    
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()

