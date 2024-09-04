arlo = robot.Robot()
tændt = 0 
print("Running ...")
print("Drej: d | Eller ligeud: l")
retning = input()
print("Left wheel: ")
leftSpeed = int(input())
print("Right wheel: ")
rightSpeed = int(input())
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
  print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
  sleep(duration)
else :
  print("sur show sunnyboy")




  
  
  
  
  
