from time import sleep

import robot

def rotate_right():
    print(arlo.go_diff(63, 63.5, 1, 0))
    sleep(0.5)
    print(arlo.stop())
# Create a robot object and initialize
arlo = robot.Robot()
leftSpeed = 63
rightSpeed = 64
print("Running ...")
for _ in range(3):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(3)
    print(arlo.stop())
    sleep(0.041)
    print(arlo.go_diff(63, 63.5, 1, 0))
    sleep(0.5)
    print(arlo.stop())
    sleep(0.041)
print("Stopping...")