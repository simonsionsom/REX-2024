from time import sleep
import robot

arlo = robot.Robot()
print("Running robot...")

leftSpeed = 40
rightSpeed = 40
thresholdDistance = 400

def stop_robot():
    arlo.stop()
    sleep(0.041)

def turn_left():
    print("Turning left")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
    sleep(1)
    stop_robot()

def turn_right():
    print("Turning right")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(1)
    stop_robot()

def go_forward():
    print("Going forwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(1)


def go_backward():
    print("Going backwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))
    sleep(1)
    stop_robot()

def fix():
    left_dist = arlo.read_left_ping_sensor()
    sleep(0.041)
    right_dist = arlo.read_right_ping_sensor()
    sleep(0.041)
    if left_dist < thresholdDistance:
        turn_right
    elif right_dist < thresholdDistance:
        turn_left
    else: None

while True:
    front_dist = arlo.read_front_ping_sensor()
    sleep(0.041)
    left_dist = arlo.read_left_ping_sensor()
    sleep(0.041)
    right_dist = arlo.read_right_ping_sensor()
    sleep(0.041)
    print(f"front_dist: {front_dist} left_dist: {left_dist} right_dist: {right_dist}")
    stop_robot()
    if front_dist < thresholdDistance:
        if left_dist > thresholdDistance:
            turn_left()
            fix()
        elif right_dist > thresholdDistance:
            turn_right()
            fix()
        else:
            go_backward()
    else:
        go_forward()
    sleep(0.041)
