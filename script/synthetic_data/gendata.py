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


def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])

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


# img = cv2.imread('/home/oongking/Research_ws/src/hole_detector/script/hikrobot/Image_Mat1.bmp',) 
img = np.full((1200,1920),0,dtype=np.uint8)
img[200:1000,440:1480] = 255
cv2.imshow('img', img)
cv2.setMouseCallback('img', click_event) 
while True:
    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('a'):
        
        break

t1_start = perf_counter() 
# process_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# print(f"img : {process_img}")
# print(f"img : {process_img.shape}")
process_img = img
# print(f"img : {process_img}")
# print(f"img : {process_img.shape}")
ret,thresh1 = cv2.threshold(process_img,127,255,cv2.THRESH_BINARY)

kernel4 = np.ones((4, 4), np.uint8)

final = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel4)

contours, hierarchy = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE )



# while True:
#     cv2.imshow('mask', resize_percent(final,50) )
#     waitkeyboard = cv2.waitKey(1)
#     if waitkeyboard & 0xFF==ord('q'):
#         break

laserPts = cv2.findNonZero(final)
print(f"laserPts : {laserPts.shape}")

homoImgPoints = np.hstack(
            (laserPts[:, 0], np.ones(laserPts.shape[0]).reshape(-1, 1),))


rays = createRays(homoImgPoints, np.linalg.inv(hikrobot_camera_matrix))
# print(f"ray : {rays[0]}")


plane = [np.array([ 0, 0, 0.7]),np.array([ 0, 0, -1])]

points3D = linePlaneIntersection(plane, rays)
# print(points3D)
objPoints = []
objPoints.extend(points3D)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.vstack(objPoints).astype(np.float64))
# print(pcd.points[0])
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
t1_stop = perf_counter()
# print("Elapsed time during the whole program in seconds:",
#                                         t1_stop-t1_start)

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

pcd = pcd.voxel_down_sample(0.003)

while True:
    cv2.imshow('image', final) 
    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('q'):
        
        break
    
    if waitkeyboard & 0xFF==ord('p'):

        # o3d.visualization.draw_geometries([pcd,Realcoor])
        
        PointWS = pick_points(pcd)
        selectpoint = []
        for i,x in enumerate(PointWS):
            selectpoint.append(np.asarray(pcd.points)[x])

        vector = np.subtract(selectpoint[0],selectpoint[1])

        print(f"Hole Diameter : {np.linalg.norm(vector)} \n\n")
        break

o3d.visualization.draw_geometries([pcd,Realcoor])
# print(o3d.__version__)

# device = o3d.core.Device("CPU:0")
# dtype = o3d.core.float32
# pcd_t = o3d.t.geometry.PointCloud(device)
# print(f"pcd.points : {np.array(pcd.points).shape}")
# pcd_t.point.positions = o3d.core.Tensor(np.array(pcd.points), dtype, device)

pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcd, o3d.core.float64)

pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)
# o3d.visualization.draw_geometries([pcd_t,Realcoor])

boundarys, mask = pcd_t.compute_boundary_points(0.01, 30)
# TODO: not good to get size of points.
print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd_t.point.positions.shape[0]} points.")

boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
pcd_t = pcd_t.paint_uniform_color([0.6, 0.6, 0.6])

boundarys_pcd = boundarys.to_legacy()
o3d.visualization.draw_geometries([ boundarys_pcd,Realcoor])

def Cluster(pcd, Clus_eps=0.01, Clus_min_points=5, Out_nb_neighbors = 20, Out_std_ratio = 1.0, point_upper_limit = 100, point_lower_limit = 3, obj_size = 0.5):
    pcds = []
    
    labels = np.array(pcd.cluster_dbscan(eps = Clus_eps, min_points = Clus_min_points, print_progress=False))
    
    max_label = labels.max()
    for i in range(0,max_label+1):
        pcdcen = pcd.select_by_index(np.argwhere(labels==i))
        # pcdcen, ind = pcdcen.remove_statistical_outlier(nb_neighbors = Out_nb_neighbors,
        #                                             std_ratio = Out_std_ratio)
        # print("pcdcen : ",np.asarray(pcdcen.points).shape)
        # print(f"pcdcen.get_max_bound() : {pcdcen.get_max_bound()}")
        # print(f"pcdcen.get_min_bound() : {pcdcen.get_min_bound()}")
        # print(f"np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) : {np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound())}")
        
        
        if point_upper_limit >np.asarray(pcdcen.points).shape[0]>point_lower_limit and np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) < obj_size:
            # o3d.visualization.draw_geometries([pcdcen])
            
            pcdcen.estimate_normals()
            pcds.append(pcdcen)

    return pcds

holes = Cluster(boundarys_pcd)

class sphere:
    def __init__(self, center, radius = 0.01 , color = ( 1, 1, 0)):
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
    
o3d.visualization.draw_geometries(holes+centers+[Realcoor])

o3d.visualization.draw_geometries(centers+[pcd,Realcoor])

plane_model, inliers = pcd.segment_plane(distance_threshold=0.005,
                                         ransac_n=3,
                                         num_iterations=1000)

# find plane and project point to plane
[a, b, c, d] = plane_model

plane_normal = np.array([a, b, c])/np.linalg.norm([a,b,c])

center_point

print(f"plane_normal : {plane_normal}")
print(f"center_point : {center_point}")


sign = np.sign(np.dot(center_point[0],plane_normal))
if sign>0:
    plane_normal *= -1

shift = np.multiply(plane_normal,0.6)

center_point = np.asarray(center_point)
shift = np.full([center_point.shape[0],3],shift)
add_point = np.add(center_point,shift)

off_set = []
for point in add_point:
    off_set.append(sphere(point).pcd)

o3d.visualization.draw_geometries(centers+off_set+[pcd,Realcoor])

cv2.destroyAllWindows() 

