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

drejet = False
def kør(d):
  LR = 0
  while True :
    F = arlo.read_front_ping_sensor()
    L = arlo.read_left_ping_sensor()
    R = arlo.read_right_ping_sensor()
    if F <= 800 :
      break
    sleep(1)
    print(arlo.go_diff(45,45,1,1))
    sleep(1)
    arlo.stop()
    if LR == 1:
      print(arlo.go_diff(45,45,1,1))
      sleep(3)
      arlo.stop()
      return print("Færdig")
  if drejet == False:
    print(arlo.go_diff(42,42,1,0))
  else :
    print(arlo.go_diff(42,42,0,1))
    LR = LR + 1
  sleep(1)
  arlo.stop()
  d = True
  kør(d)

kør(drejet)
