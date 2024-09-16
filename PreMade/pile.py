from time import sleep
import robot
import keyboard  # You will need to install this with `pip install keyboard`

# Create a robot object and initialize
arlo = robot.Robot()

# Set the minimum speed for safety
MIN_SPEED = 30
DEFAULT_LEFT_SPEED = 58
DEFAULT_RIGHT_SPEED = 64

# Function to stop the robot
def stop_robot():
    print(arlo.stop())

# Function to move the robot forward
def move_forward():
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 1, 1))

# Function to move the robot backward
def move_backward():
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 0, 0))

# Function to turn the robot left
def turn_left():
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 1, 0))

# Function to turn the robot right
def turn_right():
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 0, 1))

print("Control the robot using the arrow keys. Press 'esc' to quit.")

try:
    while True:
        if keyboard.is_pressed('up'):
            move_forward()
        elif keyboard.is_pressed('down'):
            move_backward()
        elif keyboard.is_pressed('left'):
            turn_left()
        elif keyboard.is_pressed('right'):
            turn_right()
        elif keyboard.is_pressed('space'):
            stop_robot()

        # Add a small sleep to avoid high CPU usage
        sleep(0.1)

        # Check for exit condition
        if keyboard.is_pressed('esc'):
            print("Exiting program")
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Ensure robot stops when program ends
    stop_robot()
