from time import sleep
import robot

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
    print("'w' - Move forward")
    print("'s' - Move backward")
    print("'a' - Turn left")
    print("'d' - Turn right")
    print("'space' - Stop the robot")
    print("'q' - Quit the program")

print("Running ...")
show_instructions()

try:
    while True:
        command = input("Enter command: ")

        # Get the duration of movement
        if command in ['w', 's', 'a', 'd']:
            try:
                duration = float(input("Enter duration in seconds: "))
                if duration <= 0:
                    print("Please enter a positive duration.")
                    continue
            except ValueError:
                print("Invalid input. Please enter a numeric value for duration.")
                continue

        if command == 'w':
            move_forward(duration)
        elif command == 's':
            move_backward(duration)
        elif command == 'a':
            turn_left(duration)
        elif command == 'd':
            turn_right(duration)
        elif command == ' ':
            stop_robot()
        elif command == 'q':
            print("Exiting program")
            break
        else:
            print("Invalid command, please try again.")
        
        # Optional: add a small delay for smoother control
        sleep(0.1)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Ensure the robot stops when the program exits
    stop_robot()
