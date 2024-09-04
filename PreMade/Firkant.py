from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()
tændt = 0 
print("Running ...")
while tændt > 0 :
  leftSpeed = 64
  rightSpeed = 64
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
  sleep(3)
  print(arlo.stop())
  # Wait a bit before next command
  sleep(0.041)
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
  sleep(2)
  print(arlo.stop())
  tændt = tændt -1
  #din mor
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
sleep(1)
print(arlo.stop())



