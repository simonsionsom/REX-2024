from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

tændt = 0

Front = 0
Left = 0
Right = 0
Behind = 0
stop2 = False


def Scan(F,L,R,B):
  F = arlo.read_front_ping_sensor()
  L = arlo.read_left_ping_sensor()
  R = arlo.read_right_ping_sensor()
  B = arlo.read_back_ping_sensor()
  return F, L, R, B

def kør(f,l,r,b):
  while True:
    f,l,r,b = Scan(f,l,r,b)
    if f <= 500 :
      break
    arlo.stop
    sleep(1)
    if dir =="Front":
      print(arlo.go_diff(63,63,1,1))
    else :
      print(arlo.go_diff(63,63,0,0))
  if r > l:
    print(arlo.go_diff(45,46,1,0))
  else :
    print(arlo.go_diff(45,46,0,1))
  sleep(1)
  kør(f,l,r,b)
kør(Front,Left,Right,Behind)
