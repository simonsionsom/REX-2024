from time import sleep

import PreMade.robot as robot

# Create a robot object and initialize
arlo = robot.Robot()
leftSpeed = 64
rightSpeed = 32
print("Running ...")
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
sleep(4)
print(arlo.stop())
# sleep(0.041)
# leftSpeed = 32
# rightSpeed = 64
# print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
# sleep(4)
# print(arlo.stop())
print("Stopping...")