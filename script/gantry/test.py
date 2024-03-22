import numpy as np
import open3d as o3d

pcd = o3d.geometry.PointCloud()
a = np.array([0,0,0,0,0])
c = [pcd,pcd,pcd,pcd,pcd]

process_arg = np.argwhere(a==0) 

print(f"a : {a}")
print(f"process_arg : {process_arg}")
process_arg = process_arg.reshape(-1)
print(f"process_arg : {process_arg}")

c = np.asanyarray(c)
b = c[process_arg]
print(f"b : {b}")
