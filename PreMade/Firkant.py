from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()
tændt = 8 
print("Running ...")
while tændt> 0 :
  leftSpeed = 64
  rightSpeed = 64
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
  sleep(3)
  print(arlo.stop())
  leftSpeed = 64
  rightSpeed = 32
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
  sleep(3)
  print(arlo.stop())
  tændt =-1




