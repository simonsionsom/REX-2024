from time import sleep

import robot

arlo = robot.Robot()
print("Seconds: ")
seconds = float(input())
print("Left wheel: ")
leftSpeed = float(input())
print("Right wheel: ")
rightSpeed = float(input())

def lige_ud(seconds, left, right): 
    print("Kører lige ud: ")
    print(arlo.go_diff(left, right, 1, 1))
    print(f"Distance traveled {seconds/2}")
    sleep(seconds)

def drej(seconds, left, right):
    print("Venstre: {left}, Højre: {right}")
    print("I {seconds} sekunder")
    sleep(0.041)
    print(arlo.go_diff(left, right, 1, 0))
    sleep(seconds)

lige_ud(seconds, leftSpeed, rightSpeed)


