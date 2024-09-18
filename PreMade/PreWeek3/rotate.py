from time import sleep

import PreMade.robot as robot

# Create a robot object and initialize
arlo = robot.Robot()
leftSpeed = 64
rightSpeed = 64
print("Running ...")
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
sleep(1)
print(arlo.stop())
print("Stopping...")