from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()
tændt = 0 
print("Running ...")
leftSpeed = 60
rightSpeed = 64
while tændt > 0 :
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
  sleep(3)
  print(arlo.stop())
  # Wait a bit before next command
  sleep(0.041)
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
  sleep(1)
  print(arlo.stop())
  tændt = tændt -1
  #din mor
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
sleep(1)




