from time import sleep

import robot

# Create a robot object and initialize
def rotate():
    arlo = robot.Robot()
    leftSpeed = 64
    rightSpeed = 32
    print("1 degree ...")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))  # Spin in one direction
    sleep(0.41)
    print(arlo.go_diff(32, 64))
    sleep(0.41)
rotate()