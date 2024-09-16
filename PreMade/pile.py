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
    stdscr.clear()
    stdscr.addstr(0, 0, "Press 'l' to change left wheel speed or 'r' to change right wheel speed: ")

    key = stdscr.getkey()

    if key == 'l':
        stdscr.addstr(1, 0, f"Current left speed: {left_speed}. Press '+' to increase or '-' to decrease speed.")
        while True:
            adjust_key = stdscr.getkey()
            if adjust_key == '+':
                left_speed += 1
            elif adjust_key == '-':
                left_speed -= 1
            elif adjust_key == '\n':  # Enter key to confirm
                break

            if left_speed < MIN_SPEED:
                left_speed = MIN_SPEED
            stdscr.addstr(2, 0, f"Left speed: {left_speed}     ")
            stdscr.refresh()

    elif key == 'r':
        stdscr.addstr(1, 0, f"Current right speed: {right_speed}. Press '+' to increase or '-' to decrease speed.")
        while True:
            adjust_key = stdscr.getkey()
            if adjust_key == '+':
                right_speed += 1
            elif adjust_key == '-':
                right_speed -= 1
            elif adjust_key == '\n':  # Enter key to confirm
                break

            if right_speed < MIN_SPEED:
                right_speed = MIN_SPEED
            stdscr.addstr(2, 0, f"Right speed: {right_speed}     ")
            stdscr.refresh()

    else:
        stdscr.addstr(1, 0, "Invalid input! Please try again.")

    stdscr.refresh()
    sleep(2)

def main(stdscr):
    global left_speed, right_speed
    curses.curs_set(0)  # Hide the cursor
    stdscr.nodelay(1)  # Don't block while waiting for input
    stdscr.timeout(100)  # Refresh every 100 milliseconds

    show_instructions(stdscr)

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
        elif key == ord('t'):  # Change turning speed
            change_turning_speed(stdscr)
            stdscr.clear()
            show_instructions(stdscr)
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
