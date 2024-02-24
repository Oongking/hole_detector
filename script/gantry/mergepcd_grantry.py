#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *


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
import geometry_msgs.msg 
from geometry_msgs.msg  import Pose2D

import os

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

def resize(img,scale_percent):
	# scale_percent = 50 # percent of original size
	width = int(img.shape[1] * scale_percent / 100)
	height = int(img.shape[0] * scale_percent / 100)
	dim = (width, height)
	
			# resize image
	resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
	return resized



class hikrobot_merge():
    def __init__(self):

        rospy.loginfo(":: Starting hik ::")

        self.grantry2cam = np.eye(4)
        
        self.current_pos = []
        self.received_ros_cloud = None
        self.pcd = None
        self.mono8_image = None
        self.merge_pcd = o3d.geometry.PointCloud()
        
        rospy.Subscriber("/hik/image_raw", Image, self.mono8_callback)
        rospy.Subscriber("/hik/pointcloud2", PointCloud2, self.points_callback)  
        rospy.Subscriber("/grantry_feedback", Pose2D, self.pos_callback)  
        

    def pos_callback(self,data):
        self.current_pos = np.array([data.x,data.y])

    def run(self):
        while not rospy.is_shutdown():
            if (self.pcd is not None):
                pcd = copy.deepcopy(self.pcd)
                self.pcd = None

                base2cam = self.get_feedback()
                # pcd.transform(np.linalg.inv(base2cam))
                pcd.transform(base2cam)
                self.merge_pcd += pcd
                self.merge_pcd.voxel_down_sample(voxel_size=0.001)

            break

    def get_feedback(self):
        tf_base = np.eye(4)
        tf_base[:2,3] = self.current_pos

        return np.matmul(self.current_pos,self.grantry2cam)

    def get_mergpcd(self):
        return self.merge_pcd

    def reset(self):
        self.merge_pcd = o3d.geometry.PointCloud()
        

    def points_callback(self, data):
        self.received_ros_cloud=data
        self.pcd = self.convertCloudFromRosToOpen3d()
        # if show_display and self.pcd is not None:
        #     o3d.visualization.draw_geometries([Realcoor,self.pcd])

    def mono8_callback(self, received_image):
        try:
            self.mono8_image = bridge.imgmsg_to_cv2(received_image, "mono8")
        except CvBridgeError as e:
                print(e)

    def convertCloudFromRosToOpen3d(self):
        # print(":: convertCloudFromRosToOpen3d ::")
        open3d_cloud = o3d.geometry.PointCloud()
        if self.received_ros_cloud is not None:
            # Get cloud data from ros_cloud

            field_names=[field.name for field in self.received_ros_cloud.fields]
            cloud_data = list(pc2.read_points(self.received_ros_cloud, skip_nans=True, field_names = field_names))

            
            # Check empty
            
            if len(cloud_data)==0:
                print("Converting an empty cloud")
                return None  

            # Set open3d_cloud
            # print("field_names : ",field_names)
            if "rgb" in field_names:
                IDX_RGB_IN_FIELD=3 # x, y, z, rgb
                
                # Get xyz
                xyz = [(x,y,z) for x,y,z,rgba in cloud_data ] # (why cannot put this line below rgb?)

                # Get rgb
                # Check whether int or float
                if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                    rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
                else:
                    rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

                # combine
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
                open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
            else:
                xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

            # return
            return open3d_cloud
        else:
            return None
        

rospy.init_node('merge_pcd', anonymous=True)

			
rate = rospy.Rate(10) # 10hz

pose_goal = geometry_msgs.msg.Pose()

listener = tf.TransformListener()

scanner = hikrobot_merge()
img = np.ones((50,50))
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))


def get_next_file_number(folder_path,name):
    files = os.listdir(folder_path)
    # print(name)
    file_numbers = [int(file.split('.')[0][len(name):]) for file in files if file.startswith(name) and file.endswith('.pcd')]
    if file_numbers:
        return max(file_numbers) + 1
    else:
        return 0
    
folder_path = f"/home/oongking/Research_ws/src/hole_detector/data/precision_test"
data_name = "black"
num = get_next_file_number(folder_path,f"{data_name}")

while not rospy.is_shutdown():

    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('a'):
        img = img*255
        print("scanning")
        while not rospy.is_shutdown():
            cv2.imshow('img', img)
            waitkeyboard = cv2.waitKey(1)
            scanner.run()
            if waitkeyboard & 0xFF==ord('f'):
                img = img = np.ones((50,50))
                print("stop scanning")
                break
        
    if waitkeyboard & 0xFF==ord('r'):
        scanner.reset()
        print(f"Reset Next num : {num}")

    
    if waitkeyboard & 0xFF==ord('s'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        o3d.io.write_point_cloud(f"/home/oongking/Research_ws/src/hole_detector/data/grantry/{data_name}{num}.pcd", mergpcd)
        scanner.reset()
        num +=1
        print(f"Reset Next num : {num}")


    if waitkeyboard & 0xFF==ord('q'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        break

	
    

	

# def checkUR5():
    
# if __name__ == '__main__':
#     try:
#         checkUR5()
#     except rospy.ROSInterruptException:
#         pass