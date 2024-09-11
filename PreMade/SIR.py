import random
import numpy as np
from time import sleep
import robot

# Create a robot object and initialize
arlo = robot.Robot()

sleep(1)

sample = random.uniform(0, 15)
print(sample)
def p(x,N):
    p = 0.3 * N(x;2.0,1.0) + 0.4 * N(x;5.0,2.0) + 0.3 * N(x;9.0,1.0)
    return p
