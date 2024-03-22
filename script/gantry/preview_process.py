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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import os

rospy.init_node('preview_process', anonymous=True)

on_hardware = rospy.get_param("~on_hardware",True)
scan_speed = rospy.get_param("~scan_speed",1.0)



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



class Monitor_process():
    def __init__(self):

        rospy.loginfo(":: Monitoring ::")
        self.limit_box = fixbox(np.eye(3),[0,0,0],0,x=10,y=10,z =2)

        self.world2gantry = np.eye(4)
        self.world2gantry[:3,:3] = Rx(-90)
        self.world2gantry[:3,3] = [0,0,1]
        self.gantry2world = np.linalg.inv(self.world2gantry)

        self.received_ros_cloud = None
        self.merged_pcd = o3d.geometry.PointCloud()

        rospy.Subscriber("/merge_pcd", PointCloud2, self.points_callback)  
        self.pcd_pub = rospy.Publisher("/process_pcd",PointCloud2,queue_size = 1)
        self.hole_pcd_pub = rospy.Publisher("/hole_pcd",PointCloud2,queue_size = 1)
        self.marker_pub = rospy.Publisher('/Hole_cen_marker', Marker, queue_size=10)

        self.process_tag = True

        
    def send_pcd(self,pcd):
        
        br.sendTransform((self.gantry2world[0,3],self.gantry2world[1,3],self.gantry2world[2,3]),
                         quaternion_from_matrix(self.gantry2world),rospy.Time.now(),"map","gantry_base")

        ros_pcd = self.open3d_to_ros(pcd)
        self.pcd_pub.publish(ros_pcd)

    def send_hole_pcd(self,pcd):
        
        br.sendTransform((self.gantry2world[0,3],self.gantry2world[1,3],self.gantry2world[2,3]),
                         quaternion_from_matrix(self.gantry2world),rospy.Time.now(),"map","gantry_base")

        ros_pcd = self.open3d_to_ros(pcd,color = True)
        self.hole_pcd_pub.publish(ros_pcd)


    def get_mergpcd(self):
        pcd = copy.deepcopy(self.merged_pcd)
        # self.reset()
        return pcd

    def reset(self):
        self.merged_pcd = o3d.geometry.PointCloud()
        
    def points_callback(self, data):
        self.received_ros_cloud=data
        self.merged_pcd = self.convertCloudFromRosToOpen3d()


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
    
    def open3d_to_ros(self,pcd,color = False):
        # Convert Open3D PointCloud to numpy array
        
        points = np.asarray(pcd.points)

        # Create ROS PointCloud2 message
        ros_msg = PointCloud2()
        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "gantry_base"
        ros_msg.height = 1
        ros_msg.width = len(points)
        ros_msg.is_bigendian = False
        ros_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))

        if color:
            ros_msg.fields.append(PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1))
            ros_msg.fields.append(PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1))
            ros_msg.fields.append(PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1))
            colors = np.asarray(pcd.colors)
            ros_msg.point_step = 24
            ros_msg.data = np.hstack((points, colors)).astype(np.float32).tostring()
        else:
            ros_msg.point_step = 12
            ros_msg.data = points.astype(np.float32).tostring()


        ros_msg.row_step = ros_msg.point_step * len(points)
        ros_msg.is_dense = True
        # Convert numpy arrays to binary data and add to ROS message
        return ros_msg
    
    def plot_sphere_marker(self,points):

        marker = Marker()
        marker.header.frame_id = "gantry_base"  # Change the frame_id as per your requirement
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005  # Sphere diameter
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        for point in points:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
        
        self.marker_pub.publish(marker)

        self.process_tag = False

    


class Hole_detector:
    def __init__(self):

        self.box = fixbox(np.eye(3),
                          [rospy.get_param("~box_posx",0.2),
                            rospy.get_param("~box_posy",-0.2),
                            rospy.get_param("~box_posz",0.88)]
                            ,0, # offset z
                            x = rospy.get_param("~box_sizex",0.5),
                            y = rospy.get_param("~box_sizey",0.5),
                            z = rospy.get_param("~box_sizez",0.1))


        
        self.stat_outlier_nb = rospy.get_param("~stat_outlier_nb",10)
        self.stat_outlier_std_ratio = rospy.get_param("~stat_outlier_std_ratio",4.0)

        self.plane_thres = rospy.get_param("~plane_thres",0.005)

        self.mesh_radius = rospy.get_param("~mesh_radius",0.004)
        self.mesh_hole_size = rospy.get_param("~mesh_hole_size",0.01)
        
        self.boundary_radius = rospy.get_param("~boundary_radius",0.015)
        self.boundary_max_nn = rospy.get_param("~boundary_max_nn",30)
        self.boundary_angle = rospy.get_param("~boundary_angle",70)

        self.clus_radius = rospy.get_param("~clus_radius",0.005)
        self.clus_min_points = rospy.get_param("~clus_min_points",5)
        self.clus_point_up_limit = rospy.get_param("~clus_point_up_limit",150)
        self.clus_point_low_limit = rospy.get_param("~clus_point_low_limit",60)
        self.clus_size_limit = rospy.get_param("~clus_size_limit",0.05)



    def Cluster_hole(self,pcd):

        Clus_eps = self.clus_radius 
        Clus_min_points = self.clus_min_points

        # Out_nb_neighbors = 20
        # Out_std_ratio = 1.0 

        point_upper_limit = self.clus_point_up_limit
        point_lower_limit = self.clus_point_low_limit
        obj_size = self.clus_size_limit


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

    def circle_fiting(self,points):

        x = points[:,0].T 
        y = points[:,1].T
        xy = points[:,:2]


        # print(f"x : {x.shape}")
        # print(f"y : {y.shape}")
        # print(f"xy : {xy.shape}")
        # print(f"xy : {np.power(np.linalg.norm(xy,axis=1),2)}")
        B = np.power(np.linalg.norm(xy,axis=1),2).T
        A = np.array([x*2, y*2,np.ones(x.shape[0])]).T
        # print(f"A : {A}")
        # print(f"B : {B}")
        # print(f"np.linalg.lstsq(A,B) : {np.linalg.lstsq(A,B, rcond=None)[0]}")
        ans = np.linalg.lstsq(A,B, rcond=None)[0][:3]
        r = np.sqrt(ans[2]+np.power(ans[0],2)+np.power(ans[1],2))
        # print(f"radius : {r}")
        # print(f"ans : {ans}")
        # print(f"ans : {ans[2]+np.power(ans[0],2)+np.power(ans[1],2)}")
        

        return ans[:2],r

    def find_hole(self,pcd_original):
        # Preprocess pointcloud
        # o3d.visualization.draw_geometries([pcd_original,self.box,Realcoor])

        pcd = pcd_original.crop(self.box)
        # pcd = pcd.voxel_down_sample(0.003)
        # o3d.visualization.draw_geometries([pcd,Realcoor])

        pcdcen, ind = pcd.remove_statistical_outlier(nb_neighbors = self.stat_outlier_nb,
                                                    std_ratio = self.stat_outlier_std_ratio)
        
        # o3d.visualization.draw_geometries([pcdcen,Realcoor])


        plane_model, inliers = pcdcen.segment_plane(distance_threshold=self.plane_thres,
                                         ransac_n=3,
                                         num_iterations=1000)

        pcdcen = pcdcen.select_by_index(inliers)
        xyz = np.asarray(pcdcen.points)

        # find plane and project point to plane
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        # d : distance to plan  n : Normal vector of plane
        d = (a*xyz[:,0]+b*xyz[:,1]+c*xyz[:,2]+d)/np.linalg.norm([a,b,c])
        n = np.array([a, b, c])/np.linalg.norm([a,b,c])

        plane_normals = np.full([d.shape[0],3],n)
        shift = np.multiply(plane_normals,-d.reshape(-1,1))

        pcdcen.points = o3d.utility.Vector3dVector(np.add(xyz,shift, dtype=np.float64))
        # pcdcen = pcdcen.voxel_down_sample(0.001)
        # o3d.visualization.draw_geometries([pcdcen,Realcoor])

        # Mesh Fill Hole process
        pcdcen.estimate_normals()
        normal = np.asarray(pcdcen.normals)
        normal[normal[:,2]>0] *= -1
        pcdcen.normals = o3d.utility.Vector3dVector(np.array(normal, dtype=np.float64))


        rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcdcen, o3d.utility.DoubleVector([self.mesh_radius]))
        
        # o3d.visualization.draw_geometries([pcdcen, rec_mesh])
        # o3d.visualization.draw_geometries([rec_mesh])
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(rec_mesh)
        filled = mesh_t.fill_holes(hole_size= self.mesh_hole_size)
        filled = filled.to_legacy()
        # o3d.visualization.draw_geometries([filled])
        filled = filled.subdivide_midpoint(number_of_iterations=2)
        # o3d.visualization.draw_geometries([filled])
        pcdcen.points = filled.vertices
        pcdcen = pcdcen.voxel_down_sample(0.0008)
        # o3d.visualization.draw_geometries([pcdcen])


        # find boundarys in tensor
        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen, o3d.core.float64)
        pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)

        boundarys, mask = pcd_t.compute_boundary_points(self.boundary_radius, 
                                                        self.boundary_max_nn,
                                                        angle_threshold = self.boundary_angle)

        # print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd_t.point.positions.shape[0]} points.")

        boundarys_pcd = boundarys.to_legacy()

        # o3d.visualization.draw_geometries([boundarys_pcd])

        holes = self.Cluster_hole(boundarys_pcd)


        tfm_plane = np.eye(4)
        tfm_plane[:3,:3] = rotation_matrix_from_vectors([0,0,1],n)[0]
        
        centers_pcd = []
        center_point = []
        for i,hole in enumerate(holes):
            tfm_plane[:3,3] = hole.points[0]
            inv_tfm_plane = np.linalg.inv(tfm_plane)
            hole.transform(inv_tfm_plane)
            [x,y],r = self.circle_fiting(np.array(hole.points))
            hole.transform(tfm_plane)

            if r < 0.015: # Radius Lower Than 15cm
                center = [x,y,0,1]
                center = np.matmul(tfm_plane,center)[:3]
                center_pcd = sphere(center[:3],radius = 0.001,color=[0,1,0]).pcd # Green if Fit
            else:
                center = hole.get_center()
                center_pcd = sphere(center,radius = 0.001,color=[1,1,0]).pcd # Yellow if Centroid

            center_point.append(center)
            centers_pcd.append(center_pcd)

            color = np.random.randint(10, size=3)
            holes[i].paint_uniform_color([color[0]/10, color[1]/10, color[2]/10])
        
        sum_hole = o3d.geometry.PointCloud()
        for hole in holes:
            sum_hole += hole

        # o3d.visualization.draw_geometries(holes+centers_pcd+[pcd,Realcoor])
        return center_point,sum_hole

    def linePlaneIntersection(self, plane, rayDir):
        """
        Calculate the 3D intersection between a plane and a ray, returning a 3D point.
        """
        pOrigin, pNormal = plane
        # print(f"pOrigin : {pOrigin.shape}, {pOrigin}")
        # print(f"pNormal : {pNormal.shape}, {pNormal}")
        # print(f"rayDir : {rayDir.shape}")
        d = np.dot(pOrigin, pNormal) / np.dot(rayDir, pNormal)
        return rayDir * d



rate = rospy.Rate(10) # 10hz

pose_goal = geometry_msgs.msg.Pose()

listener = tf.TransformListener()
br = tf.TransformBroadcaster()

monitor = Monitor_process()
hole_detector = Hole_detector()


Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))



while not rospy.is_shutdown():

    process_pcd = monitor.get_mergpcd()
    process_pcd = process_pcd.crop(hole_detector.box)

    if not process_pcd.is_empty():
        monitor.send_pcd(process_pcd)

        if monitor.process_tag:
            center_point,sum_hole = hole_detector.find_hole(process_pcd)

            center_point = np.asarray(center_point)
            sort_i = np.argsort(center_point[:,0])
            center_point = center_point[sort_i]

            monitor.send_hole_pcd(sum_hole)
            monitor.plot_sphere_marker(center_point)
    
    rate.sleep()

   


        
	
    
