from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()
tændt = 0 
print("Running ...")
print("Vil du justere hastighed?: j/n")
hastighed = input()
if hastighed == "j":
  print("Left wheel: ")
  leftSpeed = int(input())
  print("Right wheel: ")
  rightSpeed = int(input())
else :
  print("Sætter fart til standard lige")
  leftSpeed = 58
  rightSpeed = 64
print("Drej: d | Eller ligeud: l")
retning = input()

print("Duration: ")
duration = float(input())

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

if retning == "l":
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
  sleep(duration)
elif retning == "d" :
  print("For højre: 1 |For venstre: 0")
  drejning = int(input())
  if drejning == 1:
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(duration)
  else :
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
    sleep(duration)
else :
  print("surt show sunnyboy")




