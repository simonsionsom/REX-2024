import numpy as np
import matplotlib.pyplot as plt

class Map: 
  def __init__(self):
    self.map = np.zeros((60,80))
    self.boxSzie = 5
    self.boxesID = [(1,(0,0)),(2,(55,0)),(3,(0,75)),(4,(55,75))]
  def fillMap(self):
    for i,j in self.boxesID:
      for k in range(self.boxSzie):
        for d in range(self.boxSzie):
          m,n = j 
          self.map[m+k,n+d] = i
    return self.map
  def showMap(self):
    fig, ax = plt.subplots(figsize=(12, 8))
    cax = ax.imshow(self.map, interpolation='nearest')
    ax.xaxis.set_label_position('top')
    ax.xaxis.tick_top()
    plt.show()
