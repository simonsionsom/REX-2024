from numpy import random
import numpy as np
from time import sleep
import robot

# Create a robot object and initialize
arlo = robot.Robot()

sleep(1)

# def firstpart(standarddeviationparameter):
#     np.divide(1, np.multiply(np.sqrt(np.multiply(2,np.pi)),standarddeviationparameter))
# def nextpart():
#     np.e**(np.divide(-1,2))
# def nextnextpart():
    
# def normaldistribution():
def lige_ud(sekunder, h_left, h_right): 
    print("Kører lige ud: ")
    print(arlo.go_diff(h_left, h_right, 1, 1))
    sleep(sekunder)

def drej(sekunder, h_left, h_right):
    print("Drejer med hastighederne: ")
    print("Venstre: {h_left}, Højre: {h_right}")
    print("I {sekunder} sekunder")
    print(arlo.go_diff(h_left, h_right, 0, 1))
    sleep(sekunder)

def ping():
    # request to read sonar ping sensor
    print("Front sensor = ", arlo.read_front_ping_sensor())
    sleep(0.041)

    print("Back sensor = ", arlo.read_back_ping_sensor())
    sleep(0.041)

    print("Right sensor = ", arlo.read_right_ping_sensor())
    sleep(0.041)

    print("Left sensor = ", arlo.read_left_ping_sensor())
    sleep(0.041)



front = arlo.read_front_ping_sensor()
back = arlo.read_back_ping_sensor()
right = arlo.read_right_ping_sensor()
left = arlo.read_left_ping_sensor()


def recursivesensor():
    while True:
        front = arlo.read_front_ping_sensor()
        if front <= 500 or left <= 500 or right <= 500:
            break
        ping()
        sleep(1)
        lige_ud(1, 40, 40)
        arlo.stop()
        sleep(1)
    drej(0.5, 54, 55)
    sleep(1)
    recursivesensor()

recursivesensor()




