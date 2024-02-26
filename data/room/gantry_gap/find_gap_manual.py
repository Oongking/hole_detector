import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import open3d as o3d

tvecs = np.array([[-5.26420026e-01,  4.79727641e-01,  8.42444920e-01],
 [-5.26238671e-01,  4.31495761e-01,  8.57888765e-01],
 [-5.24915970e-01,  3.82206738e-01,  8.70163321e-01],
 [-5.25149378e-01,  3.34214894e-01,  8.85546830e-01],
 [-5.25388378e-01,  2.86808765e-01,  9.01372297e-01],
 [-5.25573496e-01,  2.39230859e-01,  9.17315676e-01],
 [-5.25564740e-01,  1.91176687e-01,  9.32226212e-01],
 [-5.25772850e-01,  1.43496917e-01,  9.47911329e-01],
 [-5.26055162e-01,  9.61547194e-02,  9.63468602e-01],
 [-5.25831363e-01,  4.84087270e-02,  9.78550713e-01],
 [-5.26265943e-01,  7.15723253e-04,  9.93728961e-01],
 [-5.26371240e-01, -4.68499964e-02,  1.00909563e+00],
 [-5.26337630e-01, -9.46080059e-02,  1.02405671e+00],
 [-5.26629617e-01, -1.42588395e-01,  1.03955529e+00],
 [-5.26915101e-01, -1.90366762e-01,  1.05451152e+00],
 [-5.27209996e-01, -2.37675869e-01,  1.06985292e+00],
 [-5.27241825e-01, -2.85276385e-01,  1.08463654e+00]])

vecs = []
for vec in tvecs[1:]:
    vec_u = (tvecs[0]-vec)/np.linalg.norm((tvecs[0]-vec))
    vecs.append(vec_u)

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a = (vec1 / np.linalg.norm(vec1)).reshape(3)
    b = (vec2 / np.linalg.norm(vec2)).reshape(3)
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    angle = np.arccos(c)
    return rotation_matrix,angle

print(f"vecs : {np.mean(vecs,axis=0)}")


rotation_matrix,_ = rotation_matrix_from_vectors([1,0,0],np.mean(vecs,axis=0))



vec_z_trans = np.matmul(rotation_matrix,[0,0,1])
print(f"vec_z_trans : {vec_z_trans}")

x_angle = 90-(rotation_matrix_from_vectors(vec_z_trans,[1,0,0])[1]/np.pi)*180
print(f"x_angle : {x_angle}")
def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.02,(0,0,0))
cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.01,(0,0,0))
cam_coor.rotate(rotation_matrix)

final_rotation_matrix = np.matmul(rotation_matrix,Rx(-x_angle))

cam_coor2 = o3d.geometry.TriangleMesh.create_coordinate_frame(0.005,(0,0,0))
cam_coor2.rotate(final_rotation_matrix)

o3d.visualization.draw_geometries([Realcoor,cam_coor,cam_coor2])

print(f"final_rotation_matrix : {final_rotation_matrix}")