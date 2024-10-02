from time import sleep

import robot

arlo = robot.Robot()

print("Starting")
while True:
    front = arlo.read_front_ping_sensor()
    sleep(0.041)
    left = arlo.read_left_ping_sensor()
    sleep(0.041)
    right = arlo.read_right_ping_sensor()
    sleep(0.041)
    back = arlo.read_back_ping_sensor()
    print(f"Front: {front}")
    print(f"Left: {left}")
    print(f"Right: {right}")
    print(f"Back: {back}")
    sleep(3)