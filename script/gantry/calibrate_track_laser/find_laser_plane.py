#!/usr/bin/env python

# Image
import cv2
import rospkg
# 3D
import open3d as o3d

import getch
import numpy as np
import copy
import sys
import os

hikrobot_camera_matrix = np.array([[1263.58549252,    0.,          969.807197  ],
                                [   0.,         1265.2997817,   661.36108893],
                                [   0.,            0.,            1.        ]])
hikrobot_distortion_coefficients = np.array([[-0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376]])

arucoParams = cv2.aruco.DetectorParameters_create() 

arucoDictA3 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

zivid_boardA3 = cv2.aruco.GridBoard_create(14, 10, 0.0216, 0.0058, arucoDictA3)

def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def sort_points_clockwise(points):
    # Calculate the centroid of the points
    # points = np.asarray(points)
    
    loop_point = []
    loop_point.append(points.pop(-1))
    

    while points != []:
        dis = []
        for point in points:
            dis.append(np.linalg.norm(np.asarray(point)-np.asarray(loop_point[-1])))

        # print(dis)
        min_index = np.argmin(dis)
        loop_point.append(points.pop(min_index))

    sorted_points = loop_point
    return list(sorted_points)

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

        cv2.imshow('img', process_img) 
        

    # checking for right mouse clicks      
    if event==cv2.EVENT_RBUTTONDOWN: 
  
        # print(x, ' ', y) 
        process_img = copy.deepcopy(img)

        if len(point_list)>=3:
            contours = [np.array([point_list], dtype=np.int32)]

            for cnt in contours:
                cv2.drawContours(process_img,[cnt],0,(255,255,255),2)
   
        cv2.imshow('img', process_img) 

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


def fitPlane(points):
    """
    Fit a plane from a set of 3D points, as described in "Least Squares Fitting of Data by Linear or Quadratic Structures".
    """
    centroid = np.mean(points, axis=0)
    xxSum = 0
    xySum = 0
    xzSum = 0
    yySum = 0
    yzSum = 0
    zzSum = 0

    for point in points:
        diff = point - centroid
        xxSum += diff[0] * diff[0]
        xySum += diff[0] * diff[1]
        xzSum += diff[0] * diff[2]
        yySum += diff[1] * diff[1]
        yzSum += diff[1] * diff[2]
        zzSum += diff[2] * diff[2]

    detX = yySum * zzSum - yzSum * yzSum
    detY = xxSum * zzSum - xzSum * xzSum
    detZ = xxSum * yySum - xySum * xySum
    detMax = max(detX, detY, detZ)

    if detMax == detX:
        normal = np.array([detX, xzSum * yzSum - xySum * zzSum, xySum * yzSum - xzSum * yySum])
    elif detMax == detY:
        normal = np.array([xzSum * yzSum - xySum * zzSum, detY, xySum * xzSum - yzSum * xxSum])
    else:
        normal = np.array([xySum * yzSum - xzSum * yySum, xySum * xzSum - yzSum * xxSum, detZ])

    normal = normal / np.linalg.norm(normal)
    origin = np.array(centroid)
    return origin, normal



plane_origin = []
plane_nor = []

start = 1
stop = 9

name = "test"
# name = "laser_v"
# name = "laser_h"
folder_path = os.path.join(rospkg.RosPack().get_path('hole_detector'), 'data','onsite','laser_calibrate')
brightness = 1
contrast = 1 
for i in range(start,stop):
    img = cv2.imread(f'{folder_path}/{i}.png',) 
    
    # Adjusts the contrast by scaling the pixel values by 2.3 

    img = cv2.addWeighted(img, contrast, np.zeros(img.shape, img.dtype), 0, brightness) 

    cv2.imshow('img', resize_percent(img,50))

    waitkeyboard = cv2.waitKey(0)
    (corners, ids,rejected)= cv2.aruco.detectMarkers(img,arucoDictA3,parameters=arucoParams)

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(img,corners,ids)
        rvec=None
        tvec=None
        _,rvec,tvec = cv2.aruco.estimatePoseBoard( corners, ids, zivid_boardA3, hikrobot_camera_matrix, hikrobot_distortion_coefficients,rvec,tvec)

        if rvec is not None and tvec is not None:
            cv2.aruco.drawAxis( img, hikrobot_camera_matrix, hikrobot_distortion_coefficients, rvec, tvec, 0.08 )
            while True:
                cv2.imshow('img', resize_percent(img,50))
                if cv2.waitKey(1) & 0xFF==ord('q'):
                    
                    break

    rot, _ = cv2.Rodrigues(rvec)

    z_vec = [0,0,1]
    plane_nor.append(np.asarray(np.matmul(rot,z_vec)).reshape(3))
    tvec = np.asarray(tvec).reshape(3)
    plane_origin.append(tvec)


print(f"plane_nor : {plane_nor}")
print(f"plane_origin : {plane_origin}")

ray_point = []

def nothing(x):
    pass

for i in range(start,stop):
    img = cv2.imread(f'{folder_path}/{i}.png',) 
    # No need undistord the ray mul with K_inv
    # img = cv2.undistort(img, hikrobot_camera_matrix, hikrobot_distortion_coefficients, None, None)

    # Adjusts the contrast by scaling the pixel values by 2.3 
    img = cv2.addWeighted(img, contrast, np.zeros(img.shape, img.dtype), 0, brightness) 

    process_img = copy.deepcopy(img)
    point_list = []
    contours = []
    cv2.imshow('img', process_img)
    cv2.setMouseCallback('img', click_event) 
    
    while True:
        cv2.imshow('img', process_img)
        waitkeyboard = cv2.waitKey(1)

        if waitkeyboard & 0xFF==ord('q'):
            break
            
        if waitkeyboard & 0xFF==ord('a'):
            cv2.destroyAllWindows()
            mask = np.zeros((process_img.shape[0], process_img.shape[1], 3), dtype=np.uint8)
            cv2.fillPoly(mask, contours, (255, 255, 255))
            cv2.imshow('process', resize_percent(mask,50)) 
            cv2.waitKey(0)
            crop_img = cv2.bitwise_and(img, mask)
            cv2.imshow('process', resize_percent(crop_img,50))
            cv2.waitKey(0)

            crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            cv2.createTrackbar("threshold","process", 0,255,nothing)
            cv2.createTrackbar("kernal","process", 1,8,nothing)

            while True:
                th_val = cv2.getTrackbarPos("threshold","process")
                kernal_value = cv2.getTrackbarPos("kernal","process")

                ret,thresh1 = cv2.threshold(crop_img,th_val,255,cv2.THRESH_BINARY)
                kernel4 = np.ones((kernal_value, kernal_value), np.uint8)

                final = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel4)

                cv2.imshow('process', resize_percent(final,50))

                waitkeyboard = cv2.waitKey(1)
                if waitkeyboard & 0xFF==ord('p'):
                    break

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
            ray_point.append(rays)

            # cv2.imshow('process', resize_percent(final,50))
            cv2.imshow('process', resize_percent(img,50))
            waitkeyboard = cv2.waitKey(0)
            break

plane_nor = np.asarray(plane_nor)
plane_origin = np.asarray(plane_origin)

print(f"plane_nor : {len(plane_nor)}")
print(f"plane_origin : {len(plane_origin)}")
print(f"ray_point : {len(ray_point)}")
points3D = []
for i in range(len(ray_point)):
    points3D.append([linePlaneIntersection([plane_origin[i],plane_nor[i]], ray) for ray in ray_point[i]])

    
# Find the corrisponding laser plane
sum_point = []
for point in points3D:
    sum_point += point
referencePoints = np.array(sum_point)
laserPlane = fitPlane(referencePoints)
print(f"rayOrigin : {laserPlane[0]}")
print(f"raynormal : {laserPlane[1]}")
np.savetxt(f'/home/oongking/Research_ws/src/hole_detector/script/gantry/calibrate_track_laser/{name}_origin_normal.out', laserPlane, delimiter=',')
load = np.loadtxt(f'/home/oongking/Research_ws/src/hole_detector/script/gantry/calibrate_track_laser/{name}_origin_normal.out', delimiter=',')

print(f"{name}_origin_normal : {load}")
# points3D_1 = [linePlaneIntersection([plane_origin[0],plane_nor[0]], ray) for ray in ray_point[0]]
# points3D_2 = [linePlaneIntersection([plane_origin[1],plane_nor[1]], ray) for ray in ray_point[1]]
# points3D_3 = [linePlaneIntersection([plane_origin[2],plane_nor[2]], ray) for ray in ray_point[2]]
# points3D_4 = [linePlaneIntersection([plane_origin[3],plane_nor[3]], ray) for ray in ray_point[3]]

# if points3D_1 is not None and points3D_2 is not None and points3D_3 is not None and points3D_4 is not None:
#     # Find the corrisponding laser plane
#     referencePoints = np.array(points3D_1 + points3D_2)
#     laserPlane = fitPlane(referencePoints)
#     print(f"rayOrigin : {laserPlane[0]}")
#     print(f"raynormal : {laserPlane[1]}")



cv2.destroyAllWindows()