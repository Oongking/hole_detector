import numpy as np


po = np.array([1,1,1])
pos = np.array([[1,1,1],
               [2,2,2],
               [3,3,3]])

print(f"sub : {np.subtract(po,pos)}")
sub = np.subtract(po,pos)

n_sub = np.linalg.norm(sub,axis=1)
print(f"n_sub : {n_sub}")
