import Movements
import robot
arlo = robot.Robot()
Move = Movements.RobotMovement(arlo, 0, 0 , 0)

Move.setData()
Move.lige_ud()
Move.setData()
for i in range(6):
  Move.drej()
