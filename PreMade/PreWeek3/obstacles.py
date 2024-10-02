from time import sleep
import PreMade.robot as robot

arlo = robot.Robot()
print("Running robot...")

leftSpeed = 44
rightSpeed = 44
thresholdDistance = 300

def stop_robot():
    arlo.stop()
    sleep(0.041)

def turn_left():
    print("Turning left")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
    sleep(1.2)
    stop_robot()

def turn_right():
    print("Turning right")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(1.2)
    stop_robot()

def go_forward():
    print("Going forwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(0.041)


def go_backward():
    print("Going backwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))
    sleep(1)
    stop_robot()

# def fix():
#     left_dist = arlo.read_left_ping_sensor()
#     sleep(0.041)
#     right_dist = arlo.read_right_ping_sensor()
#     sleep(0.041)
#     if left_dist < thresholdDistance:
#         turn_right
#     elif right_dist < thresholdDistance:
#         turn_left
#     else: None

while True:
    front_dist = arlo.read_front_ping_sensor()
    sleep(0.041)
    left_dist = arlo.read_left_ping_sensor()
    sleep(0.041)
    right_dist = arlo.read_right_ping_sensor()
    sleep(0.041)
    print(f"front_dist: {front_dist} left_dist: {left_dist} right_dist: {right_dist}")
    if front_dist < thresholdDistance:
        stop_robot()
        if left_dist > thresholdDistance and left_dist > right_dist:
            turn_left()
            # fix()
        elif right_dist > thresholdDistance and right_dist > left_dist:
            turn_right()
            # fix()
        else:
            go_backward()
            turn_left()
    else:
        go_forward()
