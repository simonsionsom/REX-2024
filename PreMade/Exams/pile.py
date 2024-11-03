import robot
import curses
from time import sleep

# Create a robot object and initialize
arlo = robot.Robot()

# Set the minimum speed for safety
MIN_SPEED = 30
DEFAULT_LEFT_SPEED = 58
DEFAULT_RIGHT_SPEED = 64

# Current speeds for both wheels
left_speed = DEFAULT_LEFT_SPEED
right_speed = DEFAULT_RIGHT_SPEED

def stop_robot():
    print(arlo.stop())

def move_forward(duration):
    print(arlo.go_diff(left_speed, right_speed, 1, 1))
    sleep(duration)
    stop_robot()

def move_backward(duration):
    print(arlo.go_diff(left_speed, right_speed, 0, 0))
    sleep(duration)
    stop_robot()

def turn_left(duration):
    print(arlo.go_diff(left_speed, right_speed, 1, 0))
    sleep(duration)
    stop_robot()

def turn_right(duration):
    print(arlo.go_diff(left_speed, right_speed, 0, 1))
    sleep(duration)
    stop_robot()

def show_instructions(stdscr):
    stdscr.addstr(0, 0, "Control the robot using the arrow keys:")
    stdscr.addstr(1, 0, "'↑' - Move forward")
    stdscr.addstr(2, 0, "'↓' - Move backward")
    stdscr.addstr(3, 0, "'←' - Turn left")
    stdscr.addstr(4, 0, "'→' - Turn right")
    stdscr.addstr(5, 0, "'space' - Stop the robot")
    stdscr.addstr(6, 0, "'t' - Change turning speed")
    stdscr.addstr(7, 0, "'esc' - Quit the program")

def change_turning_speed(stdscr):
    global left_speed, right_speed
    stdscr.nodelay(0)  # Wait for user input during this part
    stdscr.clear()
    stdscr.addstr(0, 0, "Press 'l' to change left wheel speed or 'r' to change right wheel speed: ")
    stdscr.refresh()

    key = stdscr.getch()
    
    # Debugging output to see the key captured
    stdscr.addstr(3, 0, f"Key pressed: {key}")

    if key == ord('l'):
        stdscr.addstr(1, 0, f"Current left speed: {left_speed}. Use '+' to increase or '-' to decrease speed.")
        stdscr.refresh()
        speed_change = stdscr.getch()
        if speed_change == ord('+'):
            left_speed = max(left_speed + 1, MIN_SPEED)
        elif speed_change == ord('-'):
            left_speed = max(left_speed - 1, MIN_SPEED)
        stdscr
