from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

tændt = True

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

while tændt:
  Front, Left, Right, Behind = Scan(Front,Left,Right,Behind)
  print(Front)
  print(Right, Left)
  sleep(2)
  tændt = False
