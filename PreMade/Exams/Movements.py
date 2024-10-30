from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()
tændt = 4 
print("Running ...")
print("Vil du justere hastighed?: j/n")
hastighed = input()
if hastighed == "j":
  print("Left wheel: ")
  leftSpeed = float(input())
  print("Right wheel: ")
  rightSpeed = float(input())
elif hastighed == "n" :
  print("Sætter fart til standard lige")
  leftSpeed = 58
  rightSpeed = 64
else :
  print("Så stopper legen fister")
  print("Finished")
  print(arlo.stop())
  sleep(1)

'''print("Drej: d | Eller ligeud: l")
retning = input()'''
print("Duration: ")
duration = float(input())

while tændt > 0 :
  # Wait a bit before next command
  sleep(0.041)
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
  sleep(duration)
  print(arlo.stop())
  tændt = tændt -1
  #din mor

'''
if retning == "l":
  sleep(1)
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
  sleep(duration)
  print(arlo.stop())
  print("Finished")
  sleep(1)
elif retning == "d" :
  sleep(1)
  print("For højre: 1 |For venstre: 0")
  drejning = int(input())
  if drejning == 1:
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(duration)
    print("Finished")
    sleep(1)
  elif drejning == 0:
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
    sleep(duration)

    print("Finished")
    sleep(1)
  else:
    print("Så stopper legen fister")
    print("Finished")
    print(arlo.stop())
    sleep(1)

else :
  sleep(1)
  print("surt show sunnyboy")
  print(arlo.stop())
  print("Finished")
  sleep(1)


'''