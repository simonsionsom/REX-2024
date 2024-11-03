from time import sleep
import math
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

def turn_degrees(degrees, speed):
    # Convert degrees to radians and calculate arc length
    radius = 42 / 2  # Radius for a pivot turn
    arc_length = (degrees / 360) * 2 * math.pi * radius
    time_to_turn = arc_length / 17.5  # time = distance / speed

    print(f"Turning {degrees} degrees with outer wheel at speed {speed}")
    print(f"Expected time to turn: {time_to_turn} seconds")

    # Execute turn by setting one wheel to 0 or in reverse for tighter turns
    arlo.go_diff(32, 34, 1, 0)  # Adjust directions as needed
    sleep(time_to_turn)
    arlo.stop()

# Usage
degrees = float(input("Degrees to turn: "))
speed = float(input("Speed for turning wheel: "))
wheelbase = float(input("Wheelbase (distance between wheels): "))

turn_degrees(degrees, speed, wheelbase)