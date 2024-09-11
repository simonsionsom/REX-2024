from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

tændt = 0

Front = 0
Left = 0
Right = 0
Behind = 0


def Scan(F,L,R,B):
  F = arlo.read_front_ping_sensor()
  L = arlo.read_left_ping_sensor()
  R = arlo.read_right_ping_sensor()
  B = arlo.read_back_ping_sensor()
  return F, L, R, B

while tændt <3:
  stop = False
  Front, Left, Right, Behind = Scan(Front,Left,Right,Behind)
  sleep(0.41)
  print(arlo.go_diff(63,63,1,1))
  sleep(2)
  if Front < 1000 & stop == False:
    print(arlo.go_diff(45, 46, 1, 0))
    sleep(1)
    stop = True
  if stop == True:
    Front, Left, Right, Behind = Scan(Front,Left,Right,Behind)
    if Front > Behind:
      print(arlo.go_diff(63,63,1,1))
      sleep(2)
    else: 
      print(arlo.go_diff(63,63,0,0))
    print(arlo.go_diff(45,46,0,1))
    sleep(1)
  tændt= tændt + 1

