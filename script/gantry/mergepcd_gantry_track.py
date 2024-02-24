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
from std_srvs.srv import SetBool

import os

rospy.init_node('merge_pcd', anonymous=True)

on_hardware = rospy.get_param("~on_hardware",False)

if on_hardware:
    from gantry_proxy.srv import aim_pose


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

        self.world2gantry = np.eye(4)
        self.world2gantry[:3,:3] = Rx(-90)
        self.world2gantry[:3,3] = [0,0,1]
        self.gantry2world = np.linalg.inv(self.world2gantry)

        self.cam2gantry = np.array([[-2.47323185e-03, -9.99996942e-01,  9.53512728e-07,0],
                                    [ 9.54676575e-01, -2.36085994e-03,  2.97635790e-01,0],
                                    [-2.97634878e-01,  7.37032614e-04,  9.54679494e-01,0],
                                    [0,0,0,1]])
        self.gantry2cam = np.linalg.inv(self.cam2gantry)
        self.cam2laser = np.array([[ 0.99993235 ,-0.00167084 , 0.01151127 , 0.05619813 ],
                                    [-0.00167084,  0.95873473,  0.28429725, -0.1144363 ],
                                    [-0.01151127, -0.28429725,  0.95866708,  0.03326182],
                                    [ 0.        ,  0.         , 0.         , 1.        ]])
        self.laser2cam = np.linalg.inv(self.cam2laser)
        self.gan2laser = np.matmul(self.gantry2cam,self.cam2laser)
        self.current_pos = []
        self.received_ros_cloud = None
        self.pcd = None
        self.mono8_image = None
        self.merge_pcd = o3d.geometry.PointCloud()
        
        rospy.Subscriber("/hik/image_raw", Image, self.mono8_callback)
        rospy.Subscriber("/hik/pointcloud2", PointCloud2, self.points_callback)  
        rospy.Subscriber("/gantry_feedback", Pose2D, self.pos_callback)  
        self.pcd_pub = rospy.Publisher("/merge_pcd",PointCloud2,queue_size = 1)
        

        

    def pos_callback(self,data):
        self.current_pos = np.array([data.x/1000,-data.y/1000])

    def run(self):
        while not rospy.is_shutdown():
            if (self.pcd is not None):
                pcd = copy.deepcopy(self.pcd)
                self.pcd = None

                base2cam = self.get_feedback()
                # pcd.transform(np.linalg.inv(base2cam))
                pcd.transform(base2cam)
                self.merge_pcd += pcd
                # self.merge_pcd.voxel_down_sample(voxel_size=0.0001)
                

            break

    def send_pcd(self):
        
        br.sendTransform((self.gantry2world[0,3],self.gantry2world[1,3],self.gantry2world[2,3]),
                         quaternion_from_matrix(self.gantry2world),rospy.Time.now(),"map","gantry_base")

        ros_pcd = self.open3d_to_ros()
        self.pcd_pub.publish(ros_pcd)


    def get_feedback(self):
        tf_base = np.eye(4)
        tf_base[:2,3] = self.current_pos

        return np.matmul(tf_base,self.gantry2cam)

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
    
    def open3d_to_ros(self):
        # Convert Open3D PointCloud to numpy array
        points = np.asarray(self.merge_pcd.points)
        # colors = np.asarray(self.merge_pcd.colors)
        # Create ROS PointCloud2 message
        ros_msg = PointCloud2()
        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "gantry_base"
        ros_msg.height = 1
        ros_msg.width = len(points)
        ros_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        # ros_msg.fields.append(PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1))
        # ros_msg.fields.append(PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1))
        # ros_msg.fields.append(PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1))
        ros_msg.is_bigendian = False
        ros_msg.point_step = 12
        # ros_msg.point_step = 24
        ros_msg.row_step = ros_msg.point_step * len(points)
        ros_msg.is_dense = True
        # Convert numpy arrays to binary data and add to ROS message
        ros_msg.data = points.astype(np.float32).tostring()
        # ros_msg.data = np.hstack((points, colors)).astype(np.float32).tostring()
        return ros_msg
    


class Hole_detector:
    def __init__(self):
        # self.box = fixbox(np.eye(3),[0.05, -0.2, 0.88],0,x=0.2,y=0.5,z = 0.1)

        # self.box = fixbox(np.eye(3),[0.05, -0.2, 0.88+0.59],0,x=0.2,y=0.5,z = 0.1)
        self.box = fixbox(np.eye(3),[0.2, -0.2, 0.88],0,x=0.5,y=0.5,z = 0.1)

    def Cluster_hole(self,pcd, Clus_eps=0.005, Clus_min_points=5, Out_nb_neighbors = 20, Out_std_ratio = 1.0, point_upper_limit = 100, point_lower_limit = 30, obj_size = 0.05):
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

    def find_hole(self,pcd_original):
        # Preprocess pointcloud
        o3d.visualization.draw_geometries([pcd_original,self.box,Realcoor])

        pcd = pcd_original.crop(self.box)
        # pcd = pcd.voxel_down_sample(0.003)
        o3d.visualization.draw_geometries([pcd,Realcoor])

        pcdcen, ind = pcd.remove_statistical_outlier(nb_neighbors = 10,
                                                    std_ratio = 3.0)
        
        o3d.visualization.draw_geometries([pcdcen,Realcoor])


        plane_model, inliers = pcdcen.segment_plane(distance_threshold=0.005,
                                         ransac_n=3,
                                         num_iterations=1000)

        # find plane and project point to plane
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        xyz = np.asarray(pcdcen.points)
        # d : distance to plan  n : Normal vector of plane
        d = (a*xyz[:,0]+b*xyz[:,1]+c*xyz[:,2]+d)/np.linalg.norm([a,b,c])
        n = np.array([a, b, c])/np.linalg.norm([a,b,c])

        plane_normals = np.full([d.shape[0],3],n)
        shift = np.multiply(plane_normals,-d.reshape(-1,1))

        pcdcen.points = o3d.utility.Vector3dVector(np.add(xyz,shift, dtype=np.float64))
        # pcdcen = pcdcen.voxel_down_sample(0.001)


        # find boundarys in tensor
        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen, o3d.core.float64)
        pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)
        boundarys, mask = pcd_t.compute_boundary_points(0.015, 30)
        print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd_t.point.positions.shape[0]} points.")

        boundarys_pcd = boundarys.to_legacy()

        o3d.visualization.draw_geometries([boundarys_pcd])

        holes = self.Cluster_hole(boundarys_pcd)

        centers = []
        center_point = []
        for i,hole in enumerate(holes):
            color = np.random.randint(10, size=3)
            center = hole.get_center()
            center_point.append(center)
            centers.append(sphere(center).pcd)
            holes[i].paint_uniform_color([color[0]/10, color[1]/10, color[2]/10])

        o3d.visualization.draw_geometries(holes+centers+[pcd_original,Realcoor])
        return center_point,n

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




if on_hardware:
    scanning_srv = rospy.ServiceProxy('scanning', SetBool)
    set_zero_srv = rospy.ServiceProxy('set_zero', SetBool)
    aim_pose_srv = rospy.ServiceProxy('aimming', aim_pose)

rate = rospy.Rate(10) # 10hz

pose_goal = geometry_msgs.msg.Pose()

listener = tf.TransformListener()
br = tf.TransformBroadcaster()

scanner = hikrobot_merge()
hole_detector = Hole_detector()

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
    
folder_path = f"/home/irap/SCG_ws/src/hole_detector/data/room/mock_test"
data_name = "mock1cm_"
if on_hardware:
    num = get_next_file_number(folder_path,f"{data_name}")
else:
    num = 0
    data_name = "TEST_LOCAL"
    scanner.merge_pcd = o3d.io.read_point_cloud('/home/oongking/Research_ws/src/hole_detector/data/room/mock_test/mock1cm_7.pcd')


while not rospy.is_shutdown():

    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('a'):
        img = img*255

        if on_hardware:
            scanning_srv(True)

        print("scanning")
        while not rospy.is_shutdown():
            cv2.imshow('img', img)
            waitkeyboard = cv2.waitKey(1)
            scanner.run()
            if waitkeyboard & 0xFF==ord('f'):
                img = img = np.ones((50,50))
                print("stop scanning")
                if on_hardware:
                    set_zero_srv(True)
                break
                
            if waitkeyboard & 0xFF==ord('o'):
                scanner.send_pcd()

        
    if waitkeyboard & 0xFF==ord('r'):
        scanner.reset()
        if on_hardware:
            set_zero_srv(True)
        print(f"Reset Next num : {num}")

    
    if waitkeyboard & 0xFF==ord('s'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        o3d.io.write_point_cloud(f"{folder_path}/{data_name}{num}.pcd", mergpcd)
        scanner.reset()
        num +=1
        print(f"Save {data_name}{num-1} Next num : {num}")
        
    if waitkeyboard & 0xFF==ord('p'):
        mergpcd = scanner.get_mergpcd()

        cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))

        base2cam = scanner.get_feedback()
        cam_coor.transform(base2cam)

        o3d.visualization.draw_geometries([mergpcd,Realcoor,cam_coor])


    if waitkeyboard & 0xFF==ord('q'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        break

    if waitkeyboard & 0xFF==ord('t'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])

        center_point,plane_normal = hole_detector.find_hole(mergpcd)

        center_point = np.asarray(center_point)
        sort_i = np.argsort(center_point[:,0])
        center_point = center_point[sort_i]

        cen_point = []
        for cen in center_point:
            cen_point.append(sphere([cen[0],cen[1],cen[2]],0.005).pcd)

        coors = []
        tfms = []
        
        for point in center_point:
            laser_tf = np.eye(4)
            #offset
            laser_tf[:3,3] = [point[0],point[1],point[2]]
            laser_tf[:3,:3] = scanner.gan2laser[:3,:3]
            
            laser_plane_shift = np.eye(4)
            laser_plane_shift[3,3] = scanner.gan2laser[3,3]

            plane_ref_point = np.linalg.inv(laser_tf)
            plane_ref_point = np.matmul(plane_ref_point,laser_plane_shift)

            shift = hole_detector.linePlaneIntersection([plane_ref_point[:3,3],np.matmul(plane_ref_point[:3,:3],[0,0,1])],
                                                        np.matmul(scanner.gan2laser[:3,:3],[0,0,1]))

            shift_tf = np.eye(4)
            shift_tf[:3,3] = shift
            laser_tf = np.matmul(laser_tf,shift_tf)

            laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
            laser_coor.transform(laser_tf)
            coors.append(laser_coor)
            tfms.append(laser_tf)

        off_set = []
        for tfm in tfms:
            off_set.append(sphere([tfm[0,3],tfm[1,3],tfm[2,3]]).pcd)
        
        o3d.visualization.draw_geometries(off_set+[mergpcd,Realcoor])

        o3d.visualization.draw_geometries(off_set+coors+[mergpcd,Realcoor])

        laser2eff = np.matmul(scanner.laser2cam,scanner.cam2gantry)

        for i,tfm in enumerate(tfms):
 
            tfm_go = np.matmul(tfm,laser2eff)
            # control_cartesian_arm(tfm_go[:3,3],tfm_go[:3,:3])
            if on_hardware:
                aim_pose_srv((tfm_go[0,3]*1000)-10.8,-tfm_go[1,3]*1000)
            print("going")
            o3d.visualization.draw_geometries(off_set+cen_point+[mergpcd,coors[i],Realcoor])
            out = False
            while not rospy.is_shutdown():
                img = img = np.ones((100,100))
                cv2.imshow('img', img)
                waitkeyboard = cv2.waitKey(1)
                if waitkeyboard & 0xFF==ord('g'):
                    img = img = np.ones((50,50))
                    print("Next")
                    break
                if waitkeyboard & 0xFF==ord('e'):
                    print("finish")
                    out = True
                    break
            
            if out:
                break
        
    
    if waitkeyboard & 0xFF==ord('o'):
        scanner.send_pcd()

        
	
    

	

# def checkUR5():
    
# if __name__ == '__main__':
#     try:
#         checkUR5()
#     except rospy.ROSInterruptException:
#         pass
