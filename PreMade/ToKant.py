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

""" # send a go_diff command to drive forward
leftSpeed = 58
rightSpeed = 64
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1)) """