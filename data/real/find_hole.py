#!/usr/bin/env python

# Image
import cv2

# 3D
import open3d as o3d

import getch
import numpy as np
import copy
import sys
import os
from time import perf_counter

hikrobot_camera_matrix = np.array([[1263.58549252,    0.,          969.807197  ],
                                [   0.,         1265.2997817,   661.36108893],
                                [   0.,            0.,            1.        ]])
hikrobot_distortion_coefficients = np.array([[-0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376]])

def pick_points(pcd):
    # print("")
    # print(
    #     "1) Please pick at least three correspondences using [shift + left click]"
    # )
    # print("   Press [shift + right click] to undo point picking")
    # print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

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

def createRays(pts, K_inv):
    """
    Transforms an array of points into an array of 3D rays, using the inverse intrinsics K_inv.
    """
    # We simply need to multiply per K_inv, this way we get a 3D direction.
    # We can get the points of the ray multiplying the direction with a scalar.
    
    return K_inv.dot(pts.T).T

def linePlaneIntersection(plane, rayDirs):
    """
    Calculate the 3D intersection between a plane and a ray, returning a 3D point.
    """
    pOrigin, pNormal = plane

    d = np.dot(pOrigin, pNormal) / np.tensordot(rayDirs,pNormal,axes=1)


    return (np.array(rayDirs).T * d).T

def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized


def click_event(event, x, y, flags, params): 
    global img
    size = 80
    if event == cv2.EVENT_LBUTTONDOWN: 

        for i in range(10):
            for j in range(5):
                cv2.circle(img, [x+(size*i),y+(size*j)], 26, 0 , -1)
            
        cv2.imshow('img', img) 
        

    # checking for right mouse clicks      
    if event==cv2.EVENT_RBUTTONDOWN: 
   
        cv2.imshow('img', img) 



def fixbox(rot,trans,z_offset,x = 0.4,y = 0.6 ,z = 0.1) :
    # Before rotate to canon
    
    fix_box = np.array([
    [-x/2,-y/2,z_offset],
    [-x/2, y/2,z_offset],
    [x/2, y/2,z_offset],
    [x/2,-y/2,z_offset],

    [-x/2,-y/2,z+z_offset],
    [-x/2, y/2,z+z_offset],
    [x/2,-y/2,z+z_offset],
    [x/2, y/2,z+z_offset]
    ])

    fixbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(fix_box))
    fixbox.rotate(rot,(0,0,0))
    fixbox.translate(np.asarray(trans,dtype=np.float64),relative=True)
    fixbox.color = (0, 0, 0)

    return fixbox 

box = fixbox(Rz(-45),[1.005, 0.695, 0.55],0,x=0.2,y=0.3,z = 0.3)
# pcd_original = o3d.io.read_point_cloud(f"/home/oongking/Research_ws/src/hole_detector/data/real/mergepcd_paper.pcd")
pcd_original = o3d.io.read_point_cloud(f"/home/oongking/Research_ws/src/hole_detector/data/real/mergepcd_black.pcd")
# pcd_original = o3d.io.read_point_cloud(f"/home/oongking/Research_ws/src/hole_detector/data/real/mergepcd_angle.pcd")
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))

# o3d.visualization.draw_geometries([pcd_original,Realcoor,box])
pcd = pcd_original.crop(box)

o3d.visualization.draw_geometries([pcd,Realcoor,box])

PointWS = pick_points(pcd)
selectpoint = []
for i,x in enumerate(PointWS):
    selectpoint.append(np.asarray(pcd.points)[x])
# print(f"selectpoint : {selectpoint}" )
vector = np.subtract(selectpoint[0],selectpoint[1])

print(f"Hole Diameter : {np.linalg.norm(vector)} \n\n")

pcd = pcd.voxel_down_sample(0.003)
o3d.visualization.draw_geometries([pcd,Realcoor,box])

pcdcen, ind = pcd.remove_statistical_outlier(nb_neighbors = 20,
                                                    std_ratio = 1)

# o3d.visualization.draw_geometries([pcdcen,box])

plane_model, inliers = pcdcen.segment_plane(distance_threshold=0.005,
                                         ransac_n=3,
                                         num_iterations=1000)

[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcdcen.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcdcen.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# find distance from point to plane
# |Axo + Byo+ Czo + D|/√(A^2 + B^2 + C^2).

xyz = np.asarray(pcdcen.points)
# print(f"xyz : {xyz}")


d = (a*xyz[:,0]+b*xyz[:,1]+c*xyz[:,2]+d)/np.linalg.norm([a,b,c])
# print(f"d : {d}")

# print(f"xyz shape: {xyz.shape}")
# print(f"d shape: {d.shape}")

n = np.array([a, b, c])/np.linalg.norm([a,b,c])

normals = np.full([d.shape[0],3],n)

shift = np.multiply(normals,-d.reshape(-1,1))


pcdcen.points = o3d.utility.Vector3dVector(np.add(xyz,shift, dtype=np.float64))
# o3d.visualization.draw_geometries([pcdcen,box])

pcdcen = pcdcen.voxel_down_sample(0.001)

# o3d.visualization.draw_geometries([pcdcen,box])




# PointWS = pick_points(pcd)
# selectpoint = []
# for i,x in enumerate(PointWS):
#     selectpoint.append(np.asarray(pcd.points)[x])

# vector = np.subtract(selectpoint[0],selectpoint[1])

# print(f"Hole Diameter : {np.linalg.norm(vector)} \n\n")


# o3d.visualization.draw_geometries([pcd,Realcoor])

pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen, o3d.core.float64)

pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)
# o3d.visualization.draw_geometries([pcd_t,Realcoor])

boundarys, mask = pcd_t.compute_boundary_points(0.01, 30)
# TODO: not good to get size of points.
print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd_t.point.positions.shape[0]} points.")

boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
pcd_t = pcd_t.paint_uniform_color([0.6, 0.6, 0.6])

boundarys_pcd = boundarys.to_legacy()
# o3d.visualization.draw_geometries([ boundarys_pcd])

def Cluster(pcd, Clus_eps=0.01, Clus_min_points=5, Out_nb_neighbors = 20, Out_std_ratio = 1.0, point_upper_limit = 100, point_lower_limit = 3, obj_size = 0.05):
    pcds = []
    
    labels = np.array(pcd.cluster_dbscan(eps = Clus_eps, min_points = Clus_min_points, print_progress=False))
    
    max_label = labels.max()
    for i in range(0,max_label+1):
        pcdcen = pcd.select_by_index(np.argwhere(labels==i))
        # pcdcen, ind = pcdcen.remove_statistical_outlier(nb_neighbors = Out_nb_neighbors,
        #                                             std_ratio = Out_std_ratio)
        print("pcdcen : ",np.asarray(pcdcen.points).shape)
        # print(f"pcdcen.get_max_bound() : {pcdcen.get_max_bound()}")
        # print(f"pcdcen.get_min_bound() : {pcdcen.get_min_bound()}")
        # print(f"np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) : {np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound())}")
        
        
        if point_upper_limit >np.asarray(pcdcen.points).shape[0]>point_lower_limit and np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) < obj_size:
            # o3d.visualization.draw_geometries([pcdcen])
            
            pcdcen.estimate_normals()
            pcds.append(pcdcen)

    return pcds

# holes = Cluster(boundarys_pcd) #paper
holes = Cluster(boundarys_pcd,Clus_eps=0.008)

class sphere:
    def __init__(self, center, radius = 0.005 , color = ( 1, 1, 0)):
        self.pcd = o3d.geometry.TriangleMesh.create_sphere(radius)
        self.pcd.compute_vertex_normals()
        self.pcd.translate((center[0], center[1], center[2]), relative=False)
        self.pcd.paint_uniform_color(color)

centers = []
center_point = []
for i,hole in enumerate(holes):
    color = np.random.randint(10, size=3)
    center = hole.get_center()
    center_point.append(center)
    centers.append(sphere(center).pcd)
    holes[i].paint_uniform_color([color[0]/10, color[1]/10, color[2]/10])

o3d.visualization.draw_geometries(holes+centers)

center_hole = o3d.geometry.PointCloud()
center_hole.points = o3d.utility.Vector3dVector(center_point)

# o3d.visualization.draw_geometries(centers+[pcdcen,Realcoor])
o3d.visualization.draw_geometries(holes+centers+[pcd_original,Realcoor])

pcdcen += center_hole
# o3d.visualization.draw_geometries([pcdcen])

sum_hole = o3d.geometry.PointCloud()
for i,hole in enumerate(holes):
    sum_hole += hole

# dis = []
# for i in range(14):
#     PointWS = pick_points(sum_hole)
#     selectpoint = []
#     for i,x in enumerate(PointWS):
#         selectpoint.append(np.asarray(sum_hole.points)[x])
#     # print(f"selectpoint : {selectpoint}" )
#     vector = np.subtract(selectpoint[0],selectpoint[1])

#     print(f"Hole Diameter : {np.linalg.norm(vector)} \n\n")
#     dis.append(np.linalg.norm(vector))

# print(f"dis : {dis}")
# print(f"mean dis : {np.mean(dis)}")
# print(f"median dis : {np.median(dis)}")



hole = []
for i in range(5):
    for j in range(4):
        if j != 3 or i == 2:
            if (i== 0 and j == 0) or (i== 4 and j == 0):
                pass
            else:
                hole.append([j*0.032+(0.016*(i%2)),i*0.02771,0])


hole = np.array(hole)

gen_center_hole = o3d.geometry.PointCloud()
gen_center_hole.points = o3d.utility.Vector3dVector(hole)

o3d.visualization.draw_geometries([gen_center_hole])
o3d.visualization.draw_geometries([center_hole])
o3d.visualization.draw_geometries([center_hole,gen_center_hole])


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

