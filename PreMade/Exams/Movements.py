from time import sleep
import math

class RobotMovement:
  def __init__(self, robot, sec,left,right):
    self.robot = robot
    self.seconds = sec
    self.left = left
    self.right = right
    
  def lige_ud(self, længde):
    sekunder = længde/1000*5.6
    print("Kører lige ud: ")
    print(self.robot.go_diff(self.left, self.right, 1, 1))
    sleep(sekunder)
    print(self.robot.stop())

  def drej(self, vinkel):
    sekunder = (vinkel/0.28)*0.3
    if vinkel > 0:
      print(self.arlo.go_diff(self.left, self.right, 0, 1))
      sleep(sekunder)
      print(self.robot.stop())
    else:
      print(self.arlo.go_diff(self.left, self.right, 1, 0))
      sleep(sekunder)
      print(self.robot.stop())


  def turn_degrees(self, degrees, speed, wheelbase):
    # Convert degrees to radians and calculate arc length
    radius = 42 / 2  # Radius for a pivot turn
    arc_length = (degrees / 360) * 2 * math.pi * radius
    time_to_turn = arc_length / 17.5  # time = distance / speed

    print(f"Turning {self.degrees} degrees with outer wheel at speed {self.speed}")
    print(f"Expected time to turn: {self.time_to_turn} seconds")

    # Execute turn by setting one wheel to 0 or in reverse for tighter turns
    self.arlo.go_diff(32, 34, 1, 0)  # Adjust directions as needed
    sleep(time_to_turn)
    self.arlo.stop()

    def setData(self):
      print("Seconds: ")
      self.seconds = float(input())
      print("Left wheel: ")
      self.leftSpeed = float(input())
      print("Right wheel: ")
      self.rightSpeed = float(input())

      self.degrees = float(input("Degrees to turn: "))
      self.speed = float(input("Speed for turning wheel: "))