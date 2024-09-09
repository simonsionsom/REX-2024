from time import sleep

import robot


# Create a robot object and initialize
arlo = robot.Robot()

sleep(1)

# request to read Front sonar ping sensor
print("Front sensor = ", arlo.read_front_ping_sensor())
sleep(0.041)


# request to read Back sonar ping sensor
print("Back sensor = ", arlo.read_back_ping_sensor())
sleep(0.041)

# request to read Right sonar ping sensor
print("Right sensor = ", arlo.read_right_ping_sensor())
sleep(0.041)

# request to read Left sonar ping sensor
print("Left sensor = ", arlo.read_left_ping_sensor())
sleep(0.041)

front = arlo.read_front_ping_sensor()
back = arlo.read_back_ping_sensor()
right = arlo.read_right_ping_sensor()
left = arlo.read_left_ping_sensor()

straight = left/right
print(f"Straight calculation: {straight}")
leftSpeed = 63
rightSpeed = 63.5

if (left > 2500) and (right > 2500):
    while left > 900:
        arlo.go_diff(leftSpeed, rightSpeed, 1, 0)
        sleep(0.3)
        left = arlo.read_left_ping_sensor()
        sleep(0.041)
else:
    arlo.stop()



""" # send a go_diff command to drive forward
leftSpeed = 64
rightSpeed = 64
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
sleep(1) """

