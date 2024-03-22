import numpy as np

# point = np.array([1,1,1])
# normal = np.array([-0.1,-0.1,-0.1])

# print(np.dot(point,normal))
# print(np.sign(np.dot(point,normal)))

# off = [0,0,0.0215] + np.array([0.707106,-0.707106,0])*0.004

# print(off)

# eff_moveit2cam_moveit = np.array([[ 0.00550817,  0.28519852,  0.95845264,  0.29277144],
#                                                 [-0.99994433, -0.00705541,  0.00784604,  0.0032999 ],
#                                                 [ 0.00899996, -0.9584425,   0.28514377, -0.09824983],
#                                                 [ 0.,          0.,          0.,          1.,       ]])
        
# cam2laser = np.array([[ 0.99999762,  0.00031324, -0.00215838,  0.07047039],
#                         [ 0.00031324,  0.95874521,  0.28426665, -0.11025296],
#                         [ 0.00215838, -0.28426665,  0.95874283,  0.03284858],
#                         [ 0.,          0.,          0.,          1.,        ]])

# np.linalg.inv(eff_moveit2cam_moveit)

vec = np.array([[2,0,1],
            [1,1,2],
            [3,2,0]])
sort_i = np.argsort(vec[:,2])

vec = vec[sort_i]

print(vec)