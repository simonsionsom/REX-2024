import numpy as np
import matplotlib.pyplot as plt

class Map: 
  def __init__(self):
    self.mapx = 80
    self.mapy = 60
    self.map  = []
    self.boxSzie = 5
    self.RoboSize = 10
    self.boxesID = [(1,(0,0)),(2,(55,0)),(3,(0,75)),(4,(55,75))]
    self.RoboCoords = []
    self.EnemyBoxes = []

  def ResetMap(self):
    self.map = np.zeros((self.mapy,self.mapx))
    for i,j in self.boxesID:
      for k in range(self.boxSzie):
        for d in range(self.boxSzie):
          m,n = j 
          self.map[m+k,n+d] = i
  
  def drawObject(self,y,x,ID):
    self.ResetMap()
    for k in range(self.RoboSize):
      for d in range(self.RoboSize):
        if self.map[y+(5-k)][x+(5-d)] == 0:
          if ID == []:
            self.map[y+(5-k),x+(5-d)] = -1
            self.RoboCoords.append((x,y)) 
          else:
            self.map[y+(5-k),x+(5-d)] = -1
            self.EnemyBoxes.append((ID,(y,x)))

  def showMap(self):
    fig, ax = plt.subplots(figsize=(12, 8))
    cax = ax.imshow(self.map, interpolation='nearest')
    ax.xaxis.set_label_position('top')
    ax.xaxis.tick_top()
    plt.grid(axis='both')
    plt.show()
    print(self.map)
