import robot
import curses
from time import sleep

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

def main(stdscr):
    curses.curs_set(0)  # Hide the cursor
    stdscr.nodelay(1)  # Don't block while waiting for input
    stdscr.timeout(100)  # Refresh every 100 milliseconds

    stdscr.addstr(0, 0, "Control the robot using the arrow keys:")
    stdscr.addstr(1, 0, "'↑' - Move forward")
    stdscr.addstr(2, 0, "'↓' - Move backward")
    stdscr.addstr(3, 0, "'←' - Turn left")
    stdscr.addstr(4, 0, "'→' - Turn right")
    stdscr.addstr(5, 0, "'space' - Stop the robot")
    stdscr.addstr(6, 0, "'esc' - Quit the program")

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            move_forward(1)
        elif key == curses.KEY_DOWN:
            move_backward(1)
        elif key == curses.KEY_LEFT:
            turn_left(1)
        elif key == curses.KEY_RIGHT:
            turn_right(1)
        elif key == ord(' '):  # Space bar
            stop_robot()
        elif key == 27:  # Escape key
            break

        # Optional small sleep to prevent high CPU usage
        sleep(0.1)

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        stop_robot()  # Ensure the robot stops when exiting
