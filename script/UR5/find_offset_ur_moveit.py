import numpy as np
import cv2
import open3d as o3d

def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

Rzf = np.eye(4)
Rzf[:3, :3] = Rz(180)

moveit_pos = np.array([[ 7.07185655e-01, -7.07027891e-01, -1.06007814e-04,  1.70840454e-01],
                        [ 7.07027899e-01,  7.07185647e-01,  1.06020905e-04,  9.99053453e-02],
                        [ 7.46806086e-09, -1.49926945e-04,  9.99999989e-01,  4.60495505e-01],
                        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

endeff2cam_ur = np.array([[ 0.9999342,   0.00677831, -0.00925523, -0.00454114],
                        [-0.00909176,  0.96024458, -0.27901198,  0.10191871],
                        [ 0.00699606,  0.27907777,  0.96024302,  0.07070985],
                        [ 0.,          0.,          0.,          1.        ]])

end_eff_ur = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
R, _ = cv2.Rodrigues(np.array([1.0369,2.4979,-2.5139]))
base2eff_ur = np.eye(4)
base2eff_ur[:3, :3] = R
base2eff_ur[:3, 3] = [-328.23/1000,-255.63/1000,463.71/1000]
end_eff_ur.transform(base2eff_ur)

base2cam = np.matmul(base2eff_ur,endeff2cam_ur)

end_eff_moveit = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
base2eff_moveit = np.matmul(Rzf,moveit_pos)
end_eff_moveit.transform(base2eff_moveit)

inv_base2eff_moveit = np.linalg.inv(base2eff_moveit)
eff_moveit2cam_moveit = np.matmul(inv_base2eff_moveit,base2cam)

cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
base2cam_test = np.matmul(base2eff_moveit,eff_moveit2cam_moveit)

cam_coor.transform(base2cam_test)
# cam_coor.transform(base2cam)

print(f"eff_moveit2cam_moveit : {eff_moveit2cam_moveit}")

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.3,(0,0,0))


o3d.visualization.draw_geometries([Realcoor,end_eff_ur,end_eff_moveit,cam_coor])

