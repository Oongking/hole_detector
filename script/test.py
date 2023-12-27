#!/usr/bin/env python

# Image
import cv2

# 3D
import open3d as o3d

# Utility
from utils import *
import getch
import numpy as np
import time
import copy
import sys
import os

point_list = []
contours = []

def buildPCD(rgb_image,depth_image):

    depth = o3d.geometry.Image(depth_image)
    color = o3d.geometry.Image(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1.0, depth_trunc=100.0, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, zivid_intrinsic)
    
    return pcd


def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def sort_points_clockwise(points):
    # Calculate the centroid of the points
    # points = np.asarray(points)
    
    loop_point = []
    loop_point.append(points.pop(-1))
    

    while points != []:
        dis = []
        for point in points:
            dis.append(np.linalg.norm(np.asarray(point)-np.asarray(loop_point[-1])))

        print(dis)
        min_index = np.argmin(dis)
        loop_point.append(points.pop(min_index))

    sorted_points = loop_point
    return list(sorted_points)



# function to display the coordinates of 
# of the points clicked on the image  
def click_event(event, x, y, flags, params): 
    global contours,process_img,point_list

    if event == cv2.EVENT_LBUTTONDOWN: 

        if contours != []:
            pop_tag = False
            for i,point in enumerate(point_list):
                # print(f"np.linalg.norm(cnt-[x,y]) : {np.linalg.norm(np.asarray(point)-[x,y])}")
                if np.linalg.norm(np.asarray(point)-[x,y])<5:
                    point_list.pop(i)
                    pop_tag = not pop_tag
                    break
            if not pop_tag:
                point_list.append([x,y])
        else:
            point_list.append([x,y])

        process_img = copy.deepcopy(img)

        

        if len(point_list)>=3:
            point_list = sort_points_clockwise(point_list)
            for i,point in enumerate(point_list):
                cv2.circle(process_img, point, 5, (0, 255, 0) , -1)
                cv2.putText(process_img, f"{i}", point, cv2.FONT_HERSHEY_SIMPLEX , 1, (255,255,255), 2, cv2.LINE_AA) 
                cv2.putText(process_img, f"{i}", point, cv2.FONT_HERSHEY_SIMPLEX , 1, (0,0,0), 1, cv2.LINE_AA) 

            contours = [np.array([point_list], dtype=np.int32)]

            for cnt in contours:
                cv2.drawContours(process_img,[cnt],0,(255,255,255),2)
                
                
        else:
            for i,point in enumerate(point_list):
                cv2.circle(process_img, point, 5, (0, 255, 0) , -1)
                cv2.putText(process_img, f"{i}", point, cv2.FONT_HERSHEY_SIMPLEX , 1, (255,255,255), 2, cv2.LINE_AA) 
                cv2.putText(process_img, f"{i}", point, cv2.FONT_HERSHEY_SIMPLEX , 1, (0,0,0), 1, cv2.LINE_AA) 


        cv2.imshow('image', process_img) 
        

    # checking for right mouse clicks      
    if event==cv2.EVENT_RBUTTONDOWN: 
  
        print(x, ' ', y) 
        process_img = copy.deepcopy(img)


        

        if len(point_list)>=3:
            contours = [np.array([point_list], dtype=np.int32)]

            for cnt in contours:
                cv2.drawContours(process_img,[cnt],0,(255,255,255),2)
   
        cv2.imshow('image', process_img) 

img = cv2.imread('/home/oongking/Research_ws/src/hole_detector/data/mockup/holeRGB0000.png') 
dep = np.load('/home/oongking/Research_ws/src/hole_detector/data/mockup/holeDEPTH0000.png.npy')

process_img = copy.deepcopy(img)

cv2.imshow('image', process_img) 
cv2.setMouseCallback('image', click_event) 

while True:
    cv2.imshow('image', process_img) 
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('q'):
        break
        
    if waitkeyboard & 0xFF==ord('a'):
        mask = np.zeros((process_img.shape[0], process_img.shape[1], 3), dtype=np.uint8)
        cv2.fillPoly(mask, contours, (255, 255, 255))
        cv2.imshow('process', mask) 
        cv2.waitKey(0)
        crop_img = cv2.bitwise_and(img, mask)
        cv2.imshow('process', crop_img) 
        cv2.waitKey(0)

        dep = depfromcolor(mask[:,:,0],dep)
        pcd = buildPCD(img,dep)

        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        o3d.visualization.draw_geometries([pcd,Realcoor])
        downpcd = pcd.voxel_down_sample(voxel_size=0.001)
        o3d.visualization.draw_geometries([downpcd,Realcoor])

        plane_model, inliers = downpcd.segment_plane(distance_threshold=0.001,
                                                ransac_n=3,
                                                num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = downpcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = downpcd.select_by_index(inliers, invert=True)
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud,Realcoor])

        o3d.visualization.draw_geometries([outlier_cloud,Realcoor])

        pcdcen, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors = 20,
                                                    std_ratio = 0.5)
        o3d.visualization.draw_geometries([pcdcen,Realcoor])
        
        rot = rotation_matrix_from_vectors([0,0,-1],[a,b,c])
        tran = np.array([a,b,c])*0.76
        box = fixbox(rot,tran,0)
        o3d.visualization.draw_geometries([pcdcen,box,Realcoor])
        pcdcen = pcdcen.crop(box)
        o3d.visualization.draw_geometries([pcdcen,box,Realcoor])

        holes = Cluster(pcdcen,Clus_eps = 0.005,point_upper_limit = 2000,point_lower_limit = 300,obj_size = 0.08)[1]
        centers = []
        for i,hole in enumerate(holes):
            color = np.random.randint(10, size=3)
            center = hole.get_center()
            centers.append(sphere(center).pcd)
            holes[i].paint_uniform_color([color[0]/10, color[1]/10, color[2]/10])
            
        o3d.visualization.draw_geometries(holes+centers+[Realcoor])
        
        break

cv2.destroyAllWindows() 

# while not rospy.is_shutdown():

#     print("=================================================================================")
#     print("\n:: Key command ::\n\tc : Capturing & display RGBD\n\tp : Compare build and get\n\te : Shutdown the Server")
#     key = getch.getch().lower()
#     print("key : ",key)

#     if key == 'p':
#         make_pcd = cam.buildPCD()
#         Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
#         o3d.visualization.draw_geometries([make_pcd,Realcoor])
#         make_pcd.paint_uniform_color([1, 0.706, 0])
#         pcd = cam.get_pcd()
#         o3d.visualization.draw_geometries([make_pcd,pcd,Realcoor])
#         print("make_pcd : ",make_pcd)
#         print("pcd : ",pcd)

    
#     if key == 'e':
        
#         break

# cv2.destroyAllWindows()