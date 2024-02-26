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

hikrobot_camera_matrix = np.array([[1263.58549252,    0.,          969.807197  ],
                                [   0.,         1265.2997817,   661.36108893],
                                [   0.,            0.,            1.        ]])
hikrobot_distortion_coefficients = np.array([[-0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376]])

arucoParams = cv2.aruco.DetectorParameters_create() 
arucoDictA3 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

zivid_boardA3 = cv2.aruco.GridBoard_create(14, 10, 0.0216, 0.0058, arucoDictA3)



def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  
  
gantry_pos = []
gantry_rot = []
for i in range(17):
    gantry_pos.append([(200+(50*i)),0,0])
    gantry_rot.append(cv2.Rodrigues(Rx(0))[0])
gantry_pos = np.array(gantry_pos)
gantry_rot = np.array(gantry_rot).reshape(-1,3)
print(f"gantry_pos : {gantry_pos}")
print(f"gantry_rot : {gantry_rot}")


def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def inverse_tf(tvecs,Rvecs):
    new_Tvecs = []
    new_Rvecs = []

    for (tvec,rvec) in zip(tvecs,Rvecs):
        R, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec

        # print("Transformation matrix:")
        # print(T)

        T_inv = np.linalg.inv(T)

        R_inv = T_inv[:3, :3]
        new_Tvecs.append(T_inv[:3, 3].reshape(-1, 1))

        # Convert rotation matrix to rotation vector
        rvec, _ = cv2.Rodrigues(R_inv)
        new_Rvecs.append(np.asarray(rvec).reshape(-1))
    
    new_Tvecs = np.asarray(new_Tvecs).reshape(len(new_Tvecs),3)
    new_Rvecs = np.asarray(new_Rvecs).reshape(len(new_Rvecs),3)

    return new_Tvecs,new_Rvecs

def to_tf(tvecs,Rvecs):
    T_list = []
    for (tvec,rvec) in zip(tvecs,Rvecs):
        R, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec
        T_list.append(T)

    return T_list


tvecs = []
rvecs = []

coor_list = []

for i in range(1,18):
    img = cv2.imread(f'/home/oongking/Research_ws/src/hole_detector/data/room/gantry_gap/{i}.png',) 

    brightness = 10 
    # Adjusts the contrast by scaling the pixel values by 2.3 
    contrast = 2.3  
    img = cv2.addWeighted(img, contrast, np.zeros(img.shape, img.dtype), 0, brightness) 

    # cv2.imshow('img', resize_percent(img,50))
    # waitkeyboard = cv2.waitKey(0)

    (corners, ids,rejected)= cv2.aruco.detectMarkers(img,arucoDictA3,parameters=arucoParams)

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(img,corners,ids)
        rvec=None
        tvec=None
        _,rvec,tvec = cv2.aruco.estimatePoseBoard( corners, ids, zivid_boardA3, hikrobot_camera_matrix, hikrobot_distortion_coefficients,rvec,tvec)

        if rvec is not None and tvec is not None:
            tvecs.append(tvec)
            rvecs.append(rvec)
            cv2.aruco.drawAxis( img, hikrobot_camera_matrix, hikrobot_distortion_coefficients, rvec, tvec, 0.08 )
            # while True:
            #     cv2.imshow('img', resize_percent(img,50))
            #     if cv2.waitKey(1) & 0xFF==ord('q'):
            #         Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.3,(0,0,0))
            #         cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
            #         R, _ = cv2.Rodrigues(rvec)
            #         ar_tf = np.eye(4)
            #         ar_tf[:3, :3] = R
            #         ar_tf[:3, 3] = tvec.reshape(-1)
            #         cam_coor.transform(ar_tf)
            #         coor_list.append(cam_coor)
            #         # o3d.visualization.draw_geometries([Realcoor,cam_coor])
            #         break



# o3d.visualization.draw_geometries([Realcoor]+coor_list)

tvecs = np.asarray(tvecs).reshape(17,3)
print(f"tvecs : {tvecs}")
rvecs = np.asarray(rvecs).reshape(17,3)

gantry_pos,gantry_rot = inverse_tf(gantry_pos,gantry_rot)
# gantry_tf = np.array(to_tf(gantry_pos,gantry_rot))
tvecs,rvecs = inverse_tf(tvecs,rvecs)
# vec_tf = np.array(to_tf(tvecs,rvecs))

R_off, T_off = cv2.calibrateHandEye(gantry_rot,gantry_pos,rvecs,tvecs,cv2.CALIB_HAND_EYE_PARK)
# R_off, T_off = cv2.calibrateHandEye(gantry_tf[:,:3,:3],gantry_tf[:,:3,3],vec_tf[:,:3,:3],vec_tf[:,:3,3],cv2.CALIB_HAND_EYE_DANIILIDIS)
# R_off, T_off = cv2.calibrateHandEye(rvecs,tvecs,gantry_rot,gantry_pos,cv2.CALIB_HAND_EYE_PARK)


print(f"T : {T_off}")
print(f"R : {R_off}")

endeff2cam = np.eye(4)
endeff2cam[:3, :3] = R_off
endeff2cam[:3, 3] = T_off.reshape(-1)


Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.01,(0,0,0))

# print(f"endeff2cam : {endeff2cam}")

# end_eff = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
# R, _ = cv2.Rodrigues(np.array([1.0369,2.4976,-2.5138]))
# T = np.eye(4)
# T[:3, :3] = R
# T[:3, 3] = [-328.23/1000,-255.64/1000,463.74/1000]
# end_eff.transform(T)




cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.01,(0,0,0))
cam_coor.transform(endeff2cam)



o3d.visualization.draw_geometries([Realcoor,cam_coor])


# for (tvec,rvec,robot_tvec,robot_rvec) in zip(tvecs,rvecs,robot_tvecs,robot_rvecs):
#     end_eff = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
    
#     R, _ = cv2.Rodrigues(robot_rvec)
#     base2endeff = np.eye(4)
#     base2endeff[:3, :3] = R
#     base2endeff[:3, 3] = robot_tvec.reshape(-1)

#     end_eff.transform(base2endeff)

    

#     cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
#     base2cam = np.matmul(base2endeff,endeff2cam)
#     cam_coor.transform(base2cam)


#     ar_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))

#     R, _ = cv2.Rodrigues(rvec)
#     cam2ar = np.eye(4)
#     cam2ar[:3, :3] = R
#     cam2ar[:3, 3] = tvec.reshape(-1)
#     base2ar = np.matmul(base2cam,cam2ar)
#     ar_coor.transform(base2ar)

#     print(f"base2endeff : {base2endeff}")
#     print(f"endeff2cam : {endeff2cam}")
#     print(f"cam2ar : {cam2ar}")

#     o3d.visualization.draw_geometries([Realcoor,end_eff,cam_coor,ar_coor])












