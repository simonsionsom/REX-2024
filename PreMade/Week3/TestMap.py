import numpy as np
import matplotlib as plt


np.set_printoptions(threshold=np.inf)


a = np.load('map.npy')
print(a)
plt.imshow(a)
plt.show()
