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

hikrobot_camera_matrix = np.array([  [ 1782.476318359375,        0.0,        965.43896484375], 
                                        [ 0.0,              1784.1812744140625, 590.5164184570312], 
                                        [ 0,                        0,              1]])

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
    d = np.dot(pOrigin, pNormal) / np.dot(rayDir, pNormal)
    return rayDir * d

img = cv2.imread('/home/oongking/Research_ws/src/hole_detector/script/hikrobot/Image_Mat6.bmp',) 


process_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print(f"img : {process_img.shape}")

ret,thresh1 = cv2.threshold(process_img,127,255,cv2.THRESH_BINARY)

kernel4 = np.ones((4, 4), np.uint8)

final = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel4)
laserPts = cv2.findNonZero(final)
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
print(f"ray : {rays[0]}")

y = [0,1,0]
plane_normal_vec = np.asarray(np.matmul(Rx(-18),y)).reshape(3)

plane = [np.array([0,-0.250,0]),plane_normal_vec]

points3D = [linePlaneIntersection(plane, ray) for ray in rays]
# print(points3D)
objPoints = []
objPoints.extend(points3D)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.vstack(objPoints).astype(np.float64))
print(pcd)
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
o3d.visualization.draw_geometries([pcd,Realcoor])


while True:
    cv2.imshow('image', final) 
    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('q'):
        break

cv2.destroyAllWindows() 

