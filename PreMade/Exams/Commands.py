import numpy as np
import matplotlib.pyplot as plt
import Week3.marker
import Week4.RRT_integrated
import Week3.robot
from time import sleep

arlo = Week3.robot.Robot()
print("Running robot...")

leftSpeed = 44
rightSpeed = 44
thresholdDistance = 300

def stop_robot():
    arlo.stop()
    sleep(0.041)

def turn_left():
    print("Turning left")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
    sleep(1.2)
    stop_robot()

def turn_right():
    print("Turning right")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(1.2)
    stop_robot()

def go_forward():
    print("Going forwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(0.041)

def go_backward():
    print("Going backwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))
    sleep(1)
    stop_robot()

def navigate_to_waypoints(path):
    for i in range(len(path) - 1):
        current_pos = np.array(path[i])
        next_pos = np.array(path[i + 1])

        direction = next_pos - current_pos
        distance = np.linalg.norm(direction)
        unit_direction = direction / distance  # Normalize to get direction only

        # Rotate to face the next waypoint
        angle_to_turn = np.arctan2(unit_direction[1], unit_direction[0])
        rotate_to_angle(angle_to_turn)

        # Move forward to the waypoint
        while np.linalg.norm(current_pos - next_pos) > thresholdDistance:
            go_forward()
            current_pos += unit_direction * leftSpeed * 0.041  # Adjust by speed and time to simulate movement

        stop_robot()

def rotate_to_angle(target_angle):
    # Dummy implementation of rotation to target angle
    # This function would ideally include code to measure the robot's current orientation
    print(f"Rotating to angle: {target_angle}")
    if target_angle > 0:
        turn_right()
    else:
        turn_left()
    stop_robot()

import Week4.robot_models

def main():
    path_res = 0.05

    # Use the new map loaded from "mapGod.npy"
    map = Week4.MapFromNpy('mapGod.npy')

    robot = Week4.robot_models.PointMassModel(ctrl_range=[-path_res, path_res])

    rrt = Week4.RRT(
        start=[0, 49],         # Define your start position
        goal=[90, 49],        # Define your goal position
        robot_model=robot,
        map=map,              # Use the map loaded from the file
        expand_dis=2.5,
        path_resolution=path_res,
    )
    
    show_animation = True
    metadata = dict(title="RRT Test")
    writer = Week4.FFMpegWriter(fps=15, metadata=metadata)
    fig = plt.figure()
    
    with writer.saving(fig, "rrt_test.mp4", 100):
        path = rrt.planning(animation=show_animation, writer=writer)

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
            navigate_to_waypoints(path)

            # Draw final path
            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()
                writer.grab_frame()

if __name__ == '__main__':
    main()