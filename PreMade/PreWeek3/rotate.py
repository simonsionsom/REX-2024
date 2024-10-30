from time import sleep

import Week3.robot as robot

# Create a robot object and initialize
def rotate():
    arlo = robot.Robot()
    leftSpeed = 64
    rightSpeed = 64
    print("Running ...")
    while True:
        # Set the left wheel to move forward and the right wheel to move backward
        print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))  # Spin in one direction