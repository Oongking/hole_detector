#!/usr/bin/env python

# Ros
import rospy
import rosnode
import rospkg
import tf
from tf.transformations import *


# Image
import cv2

# 3D
import open3d as o3d

# Utility
from utils import *
from ai_hole_class import *
import getch
import numpy as np
import time
import copy
import sys
import geometry_msgs.msg 
from geometry_msgs.msg  import Pose2D
from std_msgs.msg  import String
from std_srvs.srv import SetBool
from dynamic_reconfigure.server import Server 
from hole_detector.cfg import hole_config_processConfig
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point


import os

rospy.init_node('merge_pcd', anonymous=True)

on_hardware = rospy.get_param("~on_hardware",True)
scan_speed = rospy.get_param("~scan_speed",1.0)
srv_tag = rospy.get_param("~srv_tag",True)

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



class hikrobot_scanner():
    def __init__(self):

        rospy.loginfo(":: Starting hikrobot scanner ::")

        self.limit_box = fixbox(np.eye(3),[0,0,0],0,x=10,y=10,z =2)

        self.world2gantry = np.eye(4)
        self.world2gantry[:3,:3] = Rx(-90)
        self.world2gantry[:3,3] = [0,0,1]
        self.gantry2world = np.linalg.inv(self.world2gantry)

        self.cam2gantry = np.array([[-2.47323185e-03, -9.99996942e-01,  9.53512728e-07,0],
                                    [ 9.54676575e-01, -2.36085994e-03,  2.97635790e-01,0],
                                    [-2.97634878e-01,  7.37032614e-04,  9.54679494e-01,0],
                                    [0,0,0,1]])
        self.gantry2cam = np.linalg.inv(self.cam2gantry)
        calibrate_track_laser_path = os.path.join(rospkg.RosPack().get_path('hole_detector'), 'script','gantry','calibrate_track_laser')

        self.cam2laser = np.loadtxt(f'{calibrate_track_laser_path}/laser_tfm_cam2laser.out', delimiter=',')
        # self.cam2laser = np.array([[ 0.99993235 ,-0.00167084 , 0.01151127 , 0.05619813 ],
        #                             [-0.00167084,  0.95873473,  0.28429725, -0.1144363 ],
        #                             [-0.01151127, -0.28429725,  0.95866708,  0.03326182],
        #                             [ 0.        ,  0.         , 0.         , 1.        ]])
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

        self.hole_pcd_pub = rospy.Publisher("/hole_pcd",PointCloud2,queue_size = 1)
        self.marker_pub = rospy.Publisher('/Hole_cen_marker', Marker, queue_size=10)
        self.marker_text_pub = rospy.Publisher('/Hole_text_marker', MarkerArray, queue_size=10)
        self.pos_serial_pub = rospy.Publisher('/pos_serial', String, queue_size=10)
        

        

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

        ros_pcd = self.open3d_to_ros(self.merge_pcd)
        self.pcd_pub.publish(ros_pcd)

    def send_hole_pcd(self,pcd):
        
        br.sendTransform((self.gantry2world[0,3],self.gantry2world[1,3],self.gantry2world[2,3]),
                         quaternion_from_matrix(self.gantry2world),rospy.Time.now(),"map","gantry_base")

        ros_pcd = self.open3d_to_ros(pcd,color = True)
        self.hole_pcd_pub.publish(ros_pcd)

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
        text_markers = MarkerArray()
        text_marker = Marker()
        text_marker.header.frame_id = "gantry_base"  # Change the frame_id as per your requirement
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.005
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.pose.orientation.w = 1


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
        
        for i,point in enumerate(points):
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
            text_marker.id = i
            text_marker.pose.position.x = p.x
            text_marker.pose.position.y = p.y-0.005
            text_marker.pose.position.z = p.z

            text_marker.text = f"x:{p.x:.4f}, y:{p.y:.4f}"
            text_markers.markers.append(copy.deepcopy(text_marker))

        
        self.marker_pub.publish(marker)
        self.marker_text_pub.publish(text_markers)

    def sent_string_format(self,gantry_pos):
        gantry_pos = np.array(gantry_pos)*1000
        gantry_pos = np.round(gantry_pos, decimals=1)
        string_format = "$;" + ";$;".join(";".join(map(str, pos)) for pos in gantry_pos[:,:2])+";"
        self.pos_serial_pub.publish(string_format)


class Hole_detector:
    def __init__(self,model = None):
        self.dynamic_server = Server(hole_config_processConfig,self.hole_config_process_callback)

        self.last_sum_hole = o3d.geometry.PointCloud()
        self.last_center_point = []
        self.set_parameter()

        self.model = model


    def set_parameter(self):
        global data_name
        self.dynamic_server.update_configuration({"dy_file_name" :data_name })

        # self.box = fixbox(np.eye(3),[0.05, -0.2, 0.88],0,x=0.2,y=0.5,z = 0.1)
        # self.box = fixbox(np.eye(3),[0.05, -0.2, 0.88+0.59],0,x=0.2,y=0.5,z = 0.1)

        posx = rospy.get_param("~box_posx",0.2)
        self.dynamic_server.update_configuration({"dy_box_posx" :posx })
        posy = rospy.get_param("~box_posy",-0.2)
        self.dynamic_server.update_configuration({"dy_box_posy" :posy })
        posz = rospy.get_param("~box_posz",0.88)
        self.dynamic_server.update_configuration({"dy_box_posz" :posz })
        sizex = rospy.get_param("~box_sizex",0.5)
        self.dynamic_server.update_configuration({"dy_box_sizex":sizex})
        sizey = rospy.get_param("~box_sizey",0.5)
        self.dynamic_server.update_configuration({"dy_box_sizey":sizey})
        sizez = rospy.get_param("~box_sizez",0.1)
        self.dynamic_server.update_configuration({"dy_box_sizez":sizez})

        self.box = fixbox(np.eye(3),
                          [posx,
                            posy,
                            posz]
                            ,0, # offset z
                            x = sizex,
                            y = sizey,
                            z = sizez)


  
        self.stat_outlier_nb = rospy.get_param("~stat_outlier_nb",10)
        self.dynamic_server.update_configuration({"dy_stat_outlier_nb":self.stat_outlier_nb})
        self.stat_outlier_std_ratio = rospy.get_param("~stat_outlier_std_ratio",4.0)
        self.dynamic_server.update_configuration({"dy_stat_outlier_std_ratio":self.stat_outlier_std_ratio})
        
        self.plane_thres = rospy.get_param("~plane_thres",0.005)
        self.dynamic_server.update_configuration({"dy_plane_thres":self.plane_thres})

        self.mesh_radius = rospy.get_param("~mesh_radius",0.004)
        self.dynamic_server.update_configuration({"dy_mesh_radius":self.mesh_radius})
        self.mesh_hole_size = rospy.get_param("~mesh_hole_size",0.01)
        self.dynamic_server.update_configuration({"dy_mesh_hole_size":self.mesh_hole_size})

        
        self.boundary_radius = rospy.get_param("~boundary_radius",0.015)
        self.dynamic_server.update_configuration({"dy_boundary_radius":self.boundary_radius})
        self.boundary_max_nn = rospy.get_param("~boundary_max_nn",30)
        self.dynamic_server.update_configuration({"dy_boundary_max_nn":self.boundary_max_nn})
        self.boundary_angle = rospy.get_param("~boundary_angle",70)
        self.dynamic_server.update_configuration({"dy_boundary_angle":self.boundary_angle})

        self.clus_radius = rospy.get_param("~clus_radius",0.005)
        self.dynamic_server.update_configuration({"dy_clus_radius":self.clus_radius})
        self.clus_min_points = rospy.get_param("~clus_min_points",5)
        self.dynamic_server.update_configuration({"dy_clus_min_points":self.clus_min_points})
        self.clus_point_up_limit = rospy.get_param("~clus_point_up_limit",150)
        self.dynamic_server.update_configuration({"dy_clus_point_up_limit":self.clus_point_up_limit})
        self.clus_point_low_limit = rospy.get_param("~clus_point_low_limit",60)
        self.dynamic_server.update_configuration({"dy_clus_point_low_limit":self.clus_point_low_limit})
        self.clus_size_limit = rospy.get_param("~clus_size_limit",0.05)
        self.dynamic_server.update_configuration({"dy_clus_size_limit":self.clus_size_limit})


    def check_parameter(self):
        print(f"++++++++++++++++ Current Parameter ++++++++++++++++")

        print(f"stat_outlier_nb : {self.stat_outlier_nb}")
        print(f"stat_outlier_std_ratio : {self.stat_outlier_std_ratio}")
        print(f"plane_thres : {self.plane_thres}")
        print(f"mesh_radius : {self.mesh_radius}")
        print(f"mesh_hole_size : {self.mesh_hole_size}")
        print(f"boundary_radius : {self.boundary_radius}")
        print(f"boundary_max_nn : {self.boundary_max_nn}")
        print(f"boundary_angle : {self.boundary_angle}")
        print(f"clus_radius : {self.clus_radius}")
        print(f"clus_min_points : {self.clus_min_points}")
        print(f"clus_point_up_limit : {self.clus_point_up_limit}")
        print(f"clus_point_low_limit : {self.clus_point_low_limit}")
        print(f"clus_size_limit : {self.clus_size_limit}")


    def hole_config_process_callback(self,config,level) : 
        global data_name, num

        data_name = '{dy_file_name}'.format(**config)
        num = get_next_file_number(folder_path,f"{data_name}")

        posx = float('{dy_box_posx}'.format(**config))
        posy = float('{dy_box_posy}'.format(**config))
        posz = float('{dy_box_posz}'.format(**config))
        sizex = float('{dy_box_sizex}'.format(**config))
        sizey = float('{dy_box_sizey}'.format(**config))
        sizez = float('{dy_box_sizez}'.format(**config))

        self.box = fixbox(np.eye(3),
                          [posx,
                            posy,
                            posz]
                            ,0, # offset z
                            x = sizex,
                            y = sizey,
                            z = sizez)
                            
        self.stat_outlier_nb        = int('{dy_stat_outlier_nb}'.format(**config))
        self.stat_outlier_std_ratio = float('{dy_stat_outlier_std_ratio}'.format(**config))

        self.plane_thres            = float('{dy_plane_thres}'.format(**config))

        self.mesh_radius            = float('{dy_mesh_radius}'.format(**config))
        self.mesh_hole_size         = float('{dy_mesh_hole_size}'.format(**config))
        
        self.boundary_radius        = float('{dy_boundary_radius}'.format(**config))
        self.boundary_max_nn        = int('{dy_boundary_max_nn}'.format(**config))
        self.boundary_angle         = float('{dy_boundary_angle}'.format(**config))

        self.clus_radius            = float('{dy_clus_radius}'.format(**config))
        self.clus_min_points        = int('{dy_clus_min_points}'.format(**config))
        self.clus_point_up_limit    = int('{dy_clus_point_up_limit}'.format(**config))
        self.clus_point_low_limit   = int('{dy_clus_point_low_limit}'.format(**config))
        self.clus_size_limit        = float('{dy_clus_size_limit}'.format(**config))


        return config 


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
        count = 0
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
                count +=1
                pcdcen.estimate_normals()
                pcds.append(pcdcen)
        print(f"cluster count : {count}")
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
    
    def pre_ai(self,pcds,center,n):


        original_point = np.asarray(pcd.points)
        nan_rows = np.isnan(original_point).any(axis=1)

        # Remove rows with NaN values
        original_point_without_nan = original_point[~nan_rows]

        pcd.points = o3d.utility.Vector3dVector(original_point_without_nan)
        
        tfm_plane = np.eye(4)
        tfm_plane[:3,:3] = rotation_matrix_from_vectors([0,0,1],n)[0]
        tfm_plane[:3,3] = pcd.points[0]
        inv_tfm_plane = np.linalg.inv(tfm_plane)
        pcd = copy.deepcopy(pcd)
        pcd.transform(inv_tfm_plane)

        points = np.asarray(pcd.points)
        # if np.isnan(points).any():
        #     print(f"i : {i}")
        #     print(f"++++++++++++++NAN++++++++++++++")
            # o3d.visualization.draw_geometries([pcd_original])
            # o3d.visualization.draw_geometries([pcd])

            # print(f"points : {points}")

        [x,y],r = circle_fiting(points)

        tfm_center = np.eye(4)
        tfm_center[:2,3] = [x,y]
        
        inv_tfm_center = np.linalg.inv(tfm_center)
        pcd.transform(inv_tfm_center)
        return pcds
    
    def ai_classifies(self,holes,radius,hole_type):
        hole_type = np.array(hole_type)
        radius = np.array(radius)
        holes = np.asanyarray(holes)

        process_arg = np.argwhere(hole_type==1)  # 0:ai fail 1: select, 2:radius fail, 3: ai pass
        # print(f"hole_type : {hole_type}")
        # print(f"process_arg : {process_arg}")
        process_holes = holes[process_arg]
        process_radius = radius[process_arg]

        datasets = []
        for i,(p_holes,p_radius) in enumerate(zip(process_holes,process_radius)):
            # print(f"p_holes : {p_holes}")
            p_holes = p_holes[0]
            p_radius = p_radius[0]
            points = np.asarray(p_holes.points)[:,:2]*1000

            data = np.c_[points, np.full(points.shape[0],p_radius*1000)]
            num = 150-points.shape[0]
            data = np.vstack((data,np.zeros((num,3))))


            datasets.append(data)

        datasets = np.array(datasets)
        predict_data = torch.from_numpy(datasets).to(device='cuda')

        # print(f"predict_data : {predict_data.shape}")
            

        pred = model(predict_data.float())
        # print(f"pred : {pred}")

        pred = pred.argmax(1)
        ai_pred = pred.cpu().numpy()

        # print(f"b  hole_type : {hole_type}")
        hole_type[process_arg] = ai_pred.reshape(-1,1)*3 # 0 : ai fail, 3 : ai pass
        # print(f"a  hole_type : {hole_type}")


        return hole_type

        


    def find_hole(self,pcd_original):
        # Preprocess pointcloud
        # o3d.visualization.draw_geometries([pcd_original,self.box,Realcoor])

        pcd = pcd_original.crop(self.box)
        # pcd = pcd.voxel_down_sample(0.003)
        o3d.visualization.draw_geometries([pcd,Realcoor])

        pcdcen, ind = pcd.remove_statistical_outlier(nb_neighbors = self.stat_outlier_nb,
                                                    std_ratio = self.stat_outlier_std_ratio)
        
        o3d.visualization.draw_geometries([pcdcen,Realcoor])


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
        o3d.visualization.draw_geometries([pcdcen,Realcoor])

        # Mesh Fill Hole process
        pcdcen.estimate_normals()
        normal = np.asarray(pcdcen.normals)
        normal[normal[:,2]>0] *= -1
        pcdcen.normals = o3d.utility.Vector3dVector(np.array(normal, dtype=np.float64))

        print(f"Creating Mesh")
        rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcdcen, o3d.utility.DoubleVector([self.mesh_radius]))
        
        o3d.visualization.draw_geometries([pcdcen, rec_mesh],mesh_show_back_face = True)
        o3d.visualization.draw_geometries([rec_mesh],mesh_show_back_face = True)
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(rec_mesh)
        print(f"Fill Mesh Hole")
        filled = mesh_t.fill_holes(hole_size= self.mesh_hole_size)
        filled = filled.to_legacy()
        o3d.visualization.draw_geometries([filled],mesh_show_back_face = True)
        print(f"Increase Mesh vertices")

        filled = filled.subdivide_midpoint(number_of_iterations=2)
        o3d.visualization.draw_geometries([filled],mesh_show_back_face = True)
        pcdcen.points = filled.vertices
        pcdcen = pcdcen.voxel_down_sample(0.0008)
        o3d.visualization.draw_geometries([pcdcen])


        # find boundarys in tensor
        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen, o3d.core.float64)
        pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)
        pcd_t.orient_normals_consistent_tangent_plane(100)

        print(f"Compute_boundary_points")

        boundarys, mask = pcd_t.compute_boundary_points(self.boundary_radius, 
                                                        self.boundary_max_nn,
                                                        angle_threshold = self.boundary_angle)

        # print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd_t.point.positions.shape[0]} points.")

        boundarys_pcd = boundarys.to_legacy()

        o3d.visualization.draw_geometries([boundarys_pcd])

        holes = self.Cluster_hole(boundarys_pcd)


        


        tfm_plane = np.eye(4)
        tfm_plane[:3,:3] = rotation_matrix_from_vectors([0,0,1],n)[0]
        
        center_point = []
        radius = []

        hole_types = [] # 0:ai fail 1: select, 2:radius fail, 3: ai pass
        ai_holes = []
        for i,hole in enumerate(holes):
            color = np.random.randint(10, size=3)
            

            tfm_plane[:3,3] = hole.points[0]
            inv_tfm_plane = np.linalg.inv(tfm_plane)
            hole.transform(inv_tfm_plane)
            [x,y],r = self.circle_fiting(np.array(hole.points))

            hole_ai = copy.deepcopy(hole)
            hole.transform(tfm_plane)

            tfm_center = np.eye(4)
            tfm_center[:2,3] = [x,y]
            inv_tfm_center = np.linalg.inv(tfm_center)
            hole_ai.transform(inv_tfm_center)
            ai_holes.append(hole_ai)


            if r < 0.015: # Radius Lower Than 15cm
                center = [x,y,0,1]
                center = np.matmul(tfm_plane,center)[:3]
                hole_types.append(1)
            else:
                center = hole.get_center()
                hole_types.append(2)


            center_point.append(center[:3])
            radius.append(r)

        
        print(f"hole_types : {hole_types}")
        ai_pass_type = self.ai_classifies(ai_holes,radius,hole_types)
        print(f"ai_pass_type : {ai_pass_type}")

        centers_pcds = []
        sum_hole = o3d.geometry.PointCloud()
        for i,hole in enumerate(holes):
            # 0:ai fail 1: select, 2:radius fail, 3: ai pass
            if ai_pass_type[i] == 3:
                center_pcd = sphere(center_point[i],radius = 0.003,color=[0,1,0]).pcd # Green if ai pass
                hole.paint_uniform_color([0, 1, 0])
            elif ai_pass_type[i] == 2:
                center_pcd = sphere(center_point[i],radius = 0.003,color=[1,1,0]).pcd # Orange if Centroid
                hole.paint_uniform_color([1, 1, 0])
            elif ai_pass_type[i] == 1:
                center_pcd = sphere(center_point[i],radius = 0.003,color=[1,0.2,0]).pcd # Yellow if select
                hole.paint_uniform_color([1, 0.2, 0])
            elif ai_pass_type[i] == 0:
                center_pcd = sphere(center_point[i],radius = 0.003,color=[1,0,0]).pcd # Red if ai fail
                hole.paint_uniform_color([1, 0, 0])

            centers_pcds.append(center_pcd)
            sum_hole += hole

        pcd.paint_uniform_color([0, 0, 0])
        o3d.visualization.draw_geometries(centers_pcds+[pcd,sum_hole,Realcoor])

        
        # sort x
        center_point = np.asarray(center_point)
        sort_i = np.argsort(center_point[:,0])
        center_point = center_point[sort_i]

        self.last_sum_hole = copy.deepcopy(sum_hole)
        self.last_center_point = center_point

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

scanner = hikrobot_scanner()

img = np.ones((50,50))
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))


def get_next_file_number(folder_path,name):
    try:
        files = os.listdir(folder_path)
        # print(name)
        file_numbers = [int(file.split('.')[0][len(name):]) for file in files if file.startswith(name) and file.endswith('.pcd')]
    except:
        file_numbers = 0
    if file_numbers:
        return max(file_numbers) + 1
    else:
        return 0
    
folder_path = os.path.join(rospkg.RosPack().get_path('hole_detector'), 'data','onsite')

data_name = f"mock{int(scan_speed)}cm_filter_"
if on_hardware:
    num = get_next_file_number(folder_path,f"{data_name}")

else:
    num = 0
    data_name = "test_in"
    scanner.merge_pcd = o3d.io.read_point_cloud('/home/oongking/Research_ws/src/hole_detector/data/onsite/pcd/test_in1.pcd')

hole_detector = Hole_detector()

model_path = os.path.join(rospkg.RosPack().get_path('hole_detector'), 'script','gantry','weight')

model = torch.load(f"{model_path}/CNN_full_1000_best.pth").to(device='cuda')
model.eval()

while not rospy.is_shutdown():

    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('a'):
        img = img*255

        if srv_tag:
            scanning_srv(True)

        print("scanning")
        while not rospy.is_shutdown():
            cv2.imshow('img', img)
            waitkeyboard = cv2.waitKey(1)
            scanner.run()
            if waitkeyboard & 0xFF==ord('f'):
                img = img = np.ones((50,50))
                print("stop scanning")
                if srv_tag:
                    set_zero_srv(True)
                break
                
            if waitkeyboard & 0xFF==ord('o'):
                scanner.send_pcd()

        
    if waitkeyboard & 0xFF==ord('r'):
        scanner.reset()
        hole_detector.last_center_point = []
        hole_detector.last_sum_hole = o3d.geometry.PointCloud()
        
        if srv_tag:
            set_zero_srv(True)
        print(f"Reset Next num : {num}")

    
    if waitkeyboard & 0xFF==ord('s'):
        mergpcd = scanner.get_mergpcd()
        mergpcd = mergpcd.crop(scanner.limit_box)
        if not mergpcd.is_empty():
            o3d.visualization.draw_geometries([mergpcd,Realcoor])
            o3d.io.write_point_cloud(f"{folder_path}/{data_name}{num}.pcd", mergpcd)
            # scanner.reset()
            num +=1
            print(f"Save {data_name}{num-1} Next num : {num}")
            rospy.sleep(1)
        
    if waitkeyboard & 0xFF==ord('p'):
        mergpcd = scanner.get_mergpcd()
        mergpcd = mergpcd.crop(scanner.limit_box)
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        o3d.visualization.draw_geometries([mergpcd,Realcoor,hole_detector.box])

        # cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        # base2cam = scanner.get_feedback()
        # cam_coor.transform(base2cam)
        # o3d.visualization.draw_geometries([mergpcd,Realcoor,cam_coor])

    if waitkeyboard & 0xFF==ord('l'):
        mergpcd_full = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd_full,Realcoor,scanner.limit_box,hole_detector.box])
        mergpcd = mergpcd_full.crop(scanner.limit_box)
        o3d.visualization.draw_geometries([mergpcd_full,Realcoor,hole_detector.box])
        o3d.visualization.draw_geometries([mergpcd,Realcoor,hole_detector.box])

        center_point,plane_normal = hole_detector.find_hole(mergpcd)
    
    if waitkeyboard & 0xFF==ord('k'):
        hole_detector.set_parameter()
        print("Set parameter")

    if waitkeyboard & 0xFF==ord('c'):
        hole_detector.check_parameter()


    if waitkeyboard & 0xFF==ord('q'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        break

    if waitkeyboard & 0xFF==ord('h'):
        print("\n##########################")
        print("a : Scan (f : Stop)")
        print("s : Save PCD")
        print("r : Reset PCD")
        print("k : Reset parameter to launch")
        print("c : Check Parameter")
        print("p : Preview PCD")
        print("l : Preview Process")
        print("v : Preview Output")
        print("t : Process track (g : next, e: finish)")
        print("o : Publish to Rviz")
        print("q : quit")
        print("##########################")

    if waitkeyboard & 0xFF==ord('v'):
        if not hole_detector.last_sum_hole.is_empty():
            scanner.send_hole_pcd(hole_detector.last_sum_hole)
        else:
            print(f"Empty last_sum_hole PCD")
        if hole_detector.last_center_point != []:
            scanner.plot_sphere_marker(hole_detector.last_center_point)
            scanner.sent_string_format(hole_detector.last_center_point)
        else:
            print(f"Empty last_center_point List")

    if waitkeyboard & 0xFF==ord('t'):
        mergpcd_full = scanner.get_mergpcd()
        # o3d.visualization.draw_geometries([mergpcd_full,Realcoor,scanner.limit_box,hole_detector.box])
        mergpcd = mergpcd_full.crop(scanner.limit_box)
        o3d.visualization.draw_geometries([mergpcd,Realcoor,hole_detector.box])
        # o3d.visualization.draw_geometries([mergpcd,Realcoor,hole_detector.box])

        center_point,plane_normal = hole_detector.find_hole(mergpcd)

        cen_point = []
        for cen in center_point:
            cen_point.append(sphere([cen[0],cen[1],cen[2]],radius = 0.005).pcd)

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
                                                        np.array([0,0,-1]))

            shift_tf = np.eye(4)
            shift_tf[:3,3] = shift
            laser_tf = np.matmul(laser_tf,shift_tf)

            laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
            laser_coor.transform(laser_tf)
            coors.append(laser_coor)
            tfms.append(laser_tf)

        off_set = []
        for tfm in tfms:
            off_set.append(sphere([tfm[0,3],tfm[1,3],tfm[2,3]],radius = 0.005).pcd)
        
        o3d.visualization.draw_geometries(off_set+[mergpcd,Realcoor])

        o3d.visualization.draw_geometries(off_set+coors+[mergpcd,Realcoor])

        laser2eff = np.matmul(scanner.laser2cam,scanner.cam2gantry)

        for i,tfm in enumerate(tfms):
 
            tfm_go = np.matmul(tfm,laser2eff)
            # control_cartesian_arm(tfm_go[:3,3],tfm_go[:3,:3])
            if srv_tag:
                aim_pose_srv((tfm_go[0,3]*1000)-0.004,-tfm_go[1,3]*1000)
            print("going")

            laser_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.001, height=1.0)
            tf_laser_offset= np.eye(4)
            tf_laser_offset[:3,3] = [0,0,0.5]
            laser_mesh.transform(np.matmul(tfm,tf_laser_offset))
            laser_mesh.paint_uniform_color([1,0,0]) 

            tf_center= np.eye(4)
            tf_center[:3,3] = [-0.1,-0.0005,0]
            mesh_box1 = o3d.geometry.TriangleMesh.create_box(width=0.2, height=0.001, depth=1.0)
            mesh_box1.transform(np.matmul(tfm,tf_center))

            o3d.visualization.draw_geometries(off_set+cen_point+[mergpcd,coors[i],Realcoor,mesh_box1,laser_mesh])
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