from time import sleep

import robot

# Create a robot object and initialize
def rotate():
    arlo = robot.Robot()
    leftSpeed = 64
    rightSpeed = 32
    print("120 degree ....")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))  # Spin in one direction
    sleep(0.41)
    print(arlo.stop())
    sleep(0.041)
    # print("2 degree ...")
    # print(arlo.go_diff(64, 32, 0, 1))
    # sleep(1)
    # print(arlo.stop())
    # sleep(0.041)
    # print("3 degree ...")
    # print(arlo.go_diff(32, 32, 1, 0))
    # sleep(0.41)
rotate()