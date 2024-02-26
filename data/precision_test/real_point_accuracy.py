import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

import copy 

black = [[-15.69,55.48],
       [16.43,55.52],
       [-31.85,27.55],
       [0.15,27.63],
       [32.1,27.38],
       [-47.92,-0.24],
       [-15.95,0],
       [15.95,0],
       [48.12,-0.2],
       [-31.86,-28.03],
       [0.12,-27.83],
       [32.19,-28.11],
       [-15.86,-55.63],
       [16.1,-55.82],
       ]

alu = [[-16.02,55.42],
       [16.01,55.43],
       [-32.05,27.81],
       [-0.05,27.78],
       [31.85,27.44],
       [-48.06,0.05],
       [-15.97,0],
       [15.97,0],
       [47.98,-0.43],
       [-31.9,-27.67],
       [0.25,-27.58],
       [32.21,-28.2],
       [-15.63,-55.33],
       [16.37,-55.87],
       ]

black = np.array(black)/1000
alu = np.array(alu)/1000


plt.scatter(np.array(black)[:,0], np.array(black)[:,1], c='orange', marker='o')



plt.xlabel('X-axis')
plt.ylabel('Y-axis')

plt.show()


def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  
def Ry(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-np.sin(theta), 0, np.cos(theta)]])
  
def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])


# ICP

class icp_pose_estimate:
    def __init__(self,source_model,target_model,s_down = True ,t_down = True,voxel = 0.001):
        self.voxel_size = voxel

        if s_down:
            source_model = source_model.voxel_down_sample(self.voxel_size)
        if t_down:
            target_model = target_model.voxel_down_sample(self.voxel_size)

        self.source, self.source_fpfh = self.preprocess_point_cloud(source_model)
        self.target, self.target_fpfh = self.preprocess_point_cloud(target_model)
    
    def preprocess_point_cloud(self,pcd):

        radius_normal = self.voxel_size * 2
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = self.voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd, pcd_fpfh

    def estimate(self):

        distance_threshold = self.voxel_size * 1.5
        threshold = 0.2
        limit = 10

        round = 0
        while True and round <20:
            # Global estimate
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                                        self.source, self.target, self.source_fpfh, self.target_fpfh, True, distance_threshold,
                                        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, 
                                        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                                        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], 
                                        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

            # evaluate after global pose estimate
            # fitness >, inlier_rmse <
            evaluation = o3d.pipelines.registration.evaluate_registration(self.source, self.target, threshold, result.transformation)
            # print(f"=====PRE=====")
            print(f"fitness : {evaluation.fitness}")
            print(f"inlier_rmse : {evaluation.inlier_rmse}")
            
            # Point to plane estimate   need pre_tfm
            pre_tfm = result.transformation

            reg_p2p = o3d.pipelines.registration.registration_icp(
                                        self.source, self.target, threshold, pre_tfm,
                                        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200000,relative_fitness = 1e-10,relative_rmse = 1e-10))
            
            # print(f"fitness : {reg_p2p.fitness}")
            # print(f"inlier_rmse : {reg_p2p.inlier_rmse}")
            round +=1
            # if reg_p2p.fitness > 0.3 or round > limit: # or (reg_p2l.fitness == 0.0 and reg_p2l.inlier_rmse == 0.0):
            if 0.001 > reg_p2p.inlier_rmse: # or (reg_p2l.fitness == 0.0 and reg_p2l.inlier_rmse == 0.0):
                print(f"=====Result===== {round}")
                print(f"fitness : {reg_p2p.fitness}")
                print(f"inlier_rmse : {reg_p2p.inlier_rmse}")
                break
            else:
                pre_tfm = reg_p2p.transformation


        return reg_p2p.transformation,reg_p2p.inlier_rmse



black = np.hstack((black,np.zeros(black.shape[0]).reshape(-1,1)))
alu = np.hstack((alu,np.zeros(alu.shape[0]).reshape(-1,1)))

path = f"/home/oongking/Research_ws/src/hole_detector/data/precision_test/alu_cen.pcd"
center_hole = o3d.io.read_point_cloud(path)

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))


gen_center_hole = o3d.geometry.PointCloud()
gen_center_hole.points = o3d.utility.Vector3dVector(alu)
o3d.visualization.draw_geometries([gen_center_hole])
o3d.visualization.draw_geometries([gen_center_hole,center_hole,Realcoor])

center = center_hole.get_center()
tf = np.eye(4)
tf[:3,3] = center
tf[:3,:3] = Rz(-45)*Rx(90)
gen_center_hole.transform(tf)
o3d.visualization.draw_geometries([gen_center_hole,center_hole,Realcoor])

icp = icp_pose_estimate(gen_center_hole,center_hole,s_down=False,t_down=False,voxel= 0.03)
tfm,rmse = icp.estimate()

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

    return source_temp, target_temp 

gen_center_hole,center_hole =  draw_registration_result(gen_center_hole,center_hole,tfm)

gen_point = np.asarray(gen_center_hole.points)
center_hole = np.asarray(center_hole.points)

min_error = []
for cen in center_hole:
    sub = np.subtract(gen_point,cen)
    n_sub = np.linalg.norm(sub,axis=1)
    min_error.append(np.min(n_sub))

print(f"min_error : {min_error}")
print(f"mean min_error : {np.mean(min_error)}")
print(f"median min_error : {np.median(min_error)}")
print(f"max min_error : {np.max(min_error)}")
print(f"min min_error : {np.min(min_error)}")

