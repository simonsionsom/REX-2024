import robot
import keyboard  # Module for capturing keyboard input
from time import sleep

# Create a robot object and initialize
arlo = robot.Robot()

# Default speeds for moving forward and turning
leftSpeed = 58
rightSpeed = 64
turn_speed = 50  # Speed for turning

# Duration for which the robot should move on key press (can be adjusted)
movement_duration = 0.5  # Time in seconds the robot moves after a key press

def move_forward():
    print("Moving forward...")
    arlo.go_diff(leftSpeed, rightSpeed, 1, 1)
    sleep(movement_duration)
    arlo.stop()

def turn_left():
    print("Turning left...")
    arlo.go_diff(turn_speed, turn_speed, 0, 1)  # Turn left
    sleep(movement_duration)
    arlo.stop()

def turn_right():
    print("Turning right...")
    arlo.go_diff(turn_speed, turn_speed, 1, 0)  # Turn right
    sleep(movement_duration)
    arlo.stop()

print("Use 'w' to move forward, 'a' to turn left, and 'd' to turn right.")
print("Press 'q' to quit the program.")

# Main control loop
while True:
    if keyboard.is_pressed('w'):
        move_forward()
    elif keyboard.is_pressed('a'):
        turn_left()
    elif keyboard.is_pressed('d'):
        turn_right()
    elif keyboard.is_pressed('q'):
        print("Quitting the program...")
        arlo.stop()
        break

    # Small delay to prevent excessive CPU usage
    sleep(0.1)