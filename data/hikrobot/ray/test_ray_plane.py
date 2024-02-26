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
    return [np.matmul(K_inv, p) for p in pts]

def linePlaneIntersection(plane, rayDir):
    """
    Calculate the 3D intersection between a plane and a ray, returning a 3D point.
    """
    pOrigin, pNormal = plane
    # print(f"pOrigin : {pOrigin.shape}, {pOrigin}")
    # print(f"pNormal : {pNormal.shape}, {pNormal}")
    # print(f"rayDir : {rayDir.shape}")
    # print(f"np.dot(pOrigin, pNormal) : {np.dot(pOrigin, pNormal)}")
    # print(f"np.dot(rayDir, pNormal) : {np.dot(rayDir, pNormal)}")
    d = np.dot(pOrigin, pNormal) / np.dot(rayDir, pNormal)

    # print(f"rayDir : {rayDir}")
    # print(f"d : {d}")
    # print(f"rayDir * d : {rayDir * d}")
    return rayDir * d

def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized


img = cv2.imread('/home/oongking/Research_ws/src/hole_detector/script/hikrobot/Image_Mat1.bmp',) 

img = cv2.undistort(img, hikrobot_camera_matrix, hikrobot_distortion_coefficients, None, None)

t1_start = perf_counter() 
process_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# print(f"img : {process_img.shape}")

ret,thresh1 = cv2.threshold(process_img,127,255,cv2.THRESH_BINARY)

kernel4 = np.ones((4, 4), np.uint8)

final = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel4)

contours, hierarchy = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE )

# print(f"contours : {len(contours)}")
mask_final = np.zeros(final.shape)
for contour in contours:
    mask = np.zeros(final.shape)
    cv2.fillPoly(mask, [contour], (255))
    for i in range(mask.shape[1]):
        laserPts = cv2.findNonZero(mask[:,i])
        if laserPts is not None:
            # print(f"laserPts : {laserPts}")
            average = np.average(laserPts,axis = 0).reshape(-1)
            # print(f"average : {average}")
            mask_final[int(average[1]),i] = 255

    # while True:
    #     cv2.imshow('mask', resize_percent(mask_final,50) )
    #     waitkeyboard = cv2.waitKey(1)
    #     if waitkeyboard & 0xFF==ord('q'):
    #         break

laserPts = cv2.findNonZero(mask_final)
# print(f"laserPts : {laserPts.shape}")
# print(f"laserPts : {laserPts}")
# print(f"laserPts : {laserPts[:,0]}")
homoImgPoints = np.hstack(
            (laserPts[:, 0], np.ones(laserPts.shape[0]).reshape(-1, 1),))

# print(f"homoImgPoints : {homoImgPoints}")

if laserPts is not None:
    for p in laserPts:
        cv2.circle(img, (p[0][0], p[0][1]), 1, (0, 0, 255))


rays = createRays(homoImgPoints, np.linalg.inv(hikrobot_camera_matrix))
# print(f"ray : {rays[0]}")

y = [0,1,0]
plane_normal_vec = np.asarray(np.matmul(Rx(-18),y)).reshape(3)
#before distortion
# plane = [np.array([ 0.00421182, -0.00756944,  0.80403207]),np.array([ 0.00265547,  0.95242381, -0.30476522])]
#After distortion
plane = [np.array([ 0.00951941, -0.00696639,  0.80589888]),np.array([ 0.00286011,  0.95256321, -0.30432738])]

points3D = [linePlaneIntersection(plane, ray) for ray in rays]
# print(points3D)
objPoints = []
objPoints.extend(points3D)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.vstack(objPoints).astype(np.float64))
# print(pcd.points[0])
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
t1_stop = perf_counter()
print("Elapsed time during the whole program in seconds:",
                                        t1_stop-t1_start)
while True:
    cv2.imshow('image', final) 
    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('q'):
        
        break
    
    if waitkeyboard & 0xFF==ord('p'):
        o3d.visualization.draw_geometries([pcd,Realcoor])
        break


cv2.destroyAllWindows() 

