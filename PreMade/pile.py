from time import sleep
import robot
import keyboard  # Make sure you have installed this via: pip install keyboard

# Create a robot object and initialize
arlo = robot.Robot()

# Set the minimum speed for safety
MIN_SPEED = 30
DEFAULT_LEFT_SPEED = 58
DEFAULT_RIGHT_SPEED = 64

def stop_robot():
    print(arlo.stop())

def move_forward(duration):
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 1, 1))
    sleep(duration)
    stop_robot()

def move_backward(duration):
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 0, 0))
    sleep(duration)
    stop_robot()

def turn_left(duration):
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 1, 0))
    sleep(duration)
    stop_robot()

def turn_right(duration):
    print(arlo.go_diff(DEFAULT_LEFT_SPEED, DEFAULT_RIGHT_SPEED, 0, 1))
    sleep(duration)
    stop_robot()

def show_instructions():
    print("Control the robot using the following keys:")
    print("↑ (up arrow) - Move forward")
    print("↓ (down arrow) - Move backward")
    print("← (left arrow) - Turn left")
    print("→ (right arrow) - Turn right")
    print("'space' - Stop the robot")
    print("'esc' - Quit the program")

print("Running ...")
show_instructions()

try:
    while True:
        # Check for arrow key presses and move accordingly
        if keyboard.is_pressed('up'):
            print("Moving forward")
            move_forward(1)  # You can adjust the duration here

        elif keyboard.is_pressed('down'):
            print("Moving backward")
            move_backward(1)  # Adjust the duration

        elif keyboard.is_pressed('left'):
            print("Turning left")
            turn_left(1)  # Adjust the duration

        elif keyboard.is_pressed('right'):
            print("Turning right")
            turn_right(1)  # Adjust the duration

        elif keyboard.is_pressed('space'):
            print("Stopping robot")
            stop_robot()

        # Check for exit condition
        if keyboard.is_pressed('esc'):
            print("Exiting program")
            break

        # Add a small sleep to avoid high CPU usage
        sleep(0.1)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Ensure the robot stops when the program exits
    stop_robot()
