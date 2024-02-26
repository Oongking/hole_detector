import open3d as o3d
import numpy as np

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

mesh_box1 = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=0.001)


ray_origin1 = [ 0.06079354, -0.05670984,  0.43255016]
ray_normal1 = [ 0.99993374, -0.00328027, -0.011034  ]

tfm1 = np.eye(4)
z = [0,0,1]
tfm1[:3,:3] = rotation_matrix_from_vectors(z,ray_normal1)[0]
tfm1[:3,3] = np.array(ray_origin1)

mesh_box1.transform(tfm1)


mesh_box2 = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=0.001)

ray_origin2 = [ 0.117364,  -0.00601819,  0.40129452]
ray_normal2 = [ 0.0122259,   0.95861892, -0.28442977]

tfm2 = np.eye(4)
z = [0,0,1]
tfm2[:3,:3] = rotation_matrix_from_vectors(z,ray_normal2)[0]
tfm2[:3,3] = np.array(ray_origin2) 

mesh_box2.transform(tfm2)

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
laser_coor2 = o3d.geometry.TriangleMesh.create_coordinate_frame(0.025,(0,0,0))




direction = np.cross(ray_normal1, ray_normal2)

A = np.array([ray_normal1, ray_normal2,direction])
b = np.array([np.dot(ray_normal1, ray_origin1), np.dot(ray_normal2, ray_origin2),0])
print(f"A : {A}")
print(f"b : {b}")
intersection_point = np.linalg.solve(A, b)

# Print the direction and a point on the intersection line
print("Direction of intersection line:", direction)
print("Point on the intersection line:", intersection_point)
tf_laser = np.eye(4)
tf_laser[:3,:3] = rotation_matrix_from_vectors(z,direction)[0]
tf_laser[:3,3] = intersection_point
print(f"tf_cam2laser: {tf_laser}")
laser_coor.transform(tf_laser)
o3d.visualization.draw_geometries([laser_coor,Realcoor])


# np.set_printoptions(suppress=True)
# print(f"cam_laser : {np.asarray(tf_laser)}")


# eff_moveit2cam_moveit = np.array([[ 0.00411276,  0.28714067,  0.9578796,   0.29266747],
#                                                 [-0.99998082,  0.00561722,  0.00260967,  0.01017184],
#                                                 [-0.00463128, -0.95787195,  0.28715827, -0.10515685],
#                                                 [ 0.,          0.,          0.,          1.        ]])

# cam2laser = np.array([[ 0.99998571, -0.00077848,  0.00528891,  0.06550314],
#                     [-0.00077848,  0.95758911,  0.28813623, -0.10891453],
#                     [-0.00528891, -0.28813623,  0.95757482,  0.02456823],
#                     [ 0.,          0.,          0.,          1.        ]])

# cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.08,(0,0,0))
# cam_coor.transform(eff_moveit2cam_moveit)

# laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
# laser_tf = np.matmul(eff_moveit2cam_moveit,cam2laser)
# laser_coor.transform(laser_tf)

# shiff_laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.025,(0,0,0))
# shiff = np.eye(4)
# shiff[:3,3] = [0,0.003,-0.0051]

# eff_moveit2cam_moveit_shiff = np.matmul(shiff,eff_moveit2cam_moveit)
# laser_tf = np.matmul(eff_moveit2cam_moveit_shiff,cam2laser)
# shiff_laser_tf = np.matmul(laser_tf,shiff)
# shiff_laser_coor.transform(shiff_laser_tf)



# o3d.visualization.draw_geometries([shiff_laser_coor,laser_coor,cam_coor,Realcoor])


# cam2shiff_laser = np.matmul(np.linalg.inv(eff_moveit2cam_moveit),shiff_laser_tf)
# print(f"cam2shiff_laser : {cam2shiff_laser}")
