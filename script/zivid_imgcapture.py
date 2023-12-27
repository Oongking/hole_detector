#!/usr/bin/env python

# Ros
import rospy
import rosnode,rospkg
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg

# msg & convert
import geometry_msgs.msg

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


# camera = 'azure'
camera = 'zivid'

rospy.init_node("zivid_imgcap", anonymous=True)

if camera == 'azure':
    cam = Azure_cam()
    matrix_coefficients = azure_matrix_coefficients
    distortion_coefficients = azure_distortion_coefficients

if camera == 'zivid':
    cam = zivid_cam()
    matrix_coefficients = zivid_matrix_coefficients
    distortion_coefficients = zivid_distortion_coefficients

def resize(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

folder_path = os.path.join(rospkg.RosPack().get_path('hole_detector'), 'data','mockup')

def get_next_file_number(folder_path,name):
    files = os.listdir(folder_path)
    # print(name)
    file_numbers = [int(file.split('.')[0][len(name):]) for file in files if file.startswith(name) and file.endswith('.png')]
    if file_numbers:
        return max(file_numbers) + 1
    else:
        return 0
    
data_name = "hole"
num = get_next_file_number(folder_path,f"{data_name}RGB")
print(f"current num : {num:04d}")
while not rospy.is_shutdown():

    print("=================================================================================")
    print("\n:: Key command ::\n\tc : Capturing & display RGBD\n\tp : Compare build and get\n\te : Shutdown the Server")
    key = getch.getch().lower()
    print("key : ",key)

    if key == 'c':
        rgb_image, depth_image = cam.get_rgbd()
        while not rospy.is_shutdown():
            cv2.imshow("rgb_image",resize(rgb_image,50))

            waitkeyboard = cv2.waitKey(1)

            if waitkeyboard & 0xFF==ord('q'):
                print("===== End =====")
                cv2.destroyAllWindows()
                break
            if waitkeyboard & 0xFF==ord('s'):
                cv2.imwrite(folder_path+f"/{data_name}RGB{num:04d}.png",rgb_image)
                np.save(folder_path+f"/{data_name}DEPTH{num:04d}.png", depth_image)
                num+=1
                cv2.destroyAllWindows()
                break

    if key == 'p':
        make_pcd = cam.buildPCD()
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        o3d.visualization.draw_geometries([make_pcd,Realcoor])
        make_pcd.paint_uniform_color([1, 0.706, 0])
        pcd = cam.get_pcd()
        o3d.visualization.draw_geometries([make_pcd,pcd,Realcoor])
        print("make_pcd : ",make_pcd)
        print("pcd : ",pcd)

    if key == 't':
        pcd,rgb_image, depth_image,make_pcd = cam.testMatrix()

        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        # o3d.visualization.draw_geometries([make_pcd,Realcoor])
        make_pcd.paint_uniform_color([1, 0.706, 0])
        # pcd = cam.get_pcd()
        o3d.visualization.draw_geometries([make_pcd,pcd,Realcoor])
        print("make_pcd : ",make_pcd)
        print("pcd : ",pcd)
    
    if key == 'e':
        
        break

cv2.destroyAllWindows()