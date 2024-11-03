import Movements
import robot
arlo = robot.Robot()
Move = Movements.RobotMovement(arlo, 0, 0 , 0)

Move.setData()
Move.lige_ud()
Move.setData()
ib = 12
while ib > 0:
    Move.drej()
    ib = ib -1