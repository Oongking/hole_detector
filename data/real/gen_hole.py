import numpy as np
import matplotlib.pyplot as plt

hole = []
for i in range(5):
    for j in range(4):
        if j != 3 or i == 2:
            if (i== 0 and j == 0) or (i== 4 and j == 0):
                pass
            else:
                hole.append([j*0.032+(0.016*(i%2)),i*0.02771])




plt.scatter(np.array(hole)[:,0], np.array(hole)[:,1], c='orange', marker='o')



plt.xlabel('X-axis')
plt.ylabel('Y-axis')

plt.show()