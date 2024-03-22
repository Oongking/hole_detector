#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg

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

        rospy.Subscriber("/hik/image_raw", Image, self.mono8_callback)
        rospy.Subscriber("/hik/pointcloud2", PointCloud2, self.points_callback)  

        self.eff_moveit2cam_moveit = np.array([[ 0.00411276,  0.28714067,  0.9578796,   0.29266747],
                                                [-0.99998082,  0.00561722,  0.00260967,  0.01017184],
                                                [-0.00463128, -0.95787195,  0.28715827, -0.10515685],
                                                [ 0.,          0.,          0.,          1.        ]])
        # self.eff_moveit2cam_moveit = np.eye(4)
        
        # from plane
        # self.cam2laser = np.array([[ 0.99999762,  0.00031324, -0.00215838,  0.07047039],
        #                             [ 0.00031324,  0.95874521,  0.28426665, -0.11025296],
        #                             [ 0.00215838, -0.28426665,  0.95874283,  0.03284858],
        #                             [ 0.,          0.,          0.,          1.,        ]])
        # from point
        # self.cam2laser = np.array([[ 0.99998571, -0.00077848,  0.00528891,  0.06550314],
        #                             [-0.00077848,  0.95758911,  0.28813623, -0.10891453],
        #                             [-0.00528891, -0.28813623,  0.95757482,  0.02456823],
        #                             [ 0.,          0.,          0.,          1.        ]])
        # from shiff
        self.cam2laser = np.array([[ 0.99998571, -0.00077848,  0.00528891,  0.06249751],
                                    [-0.00077848,  0.95758911,  0.28813623, -0.10260926],
                                    [-0.00528891, -0.28813623,  0.95757482,  0.01736351],
                                    [ 0.,          0.,          0.,          1.        ]])


        self.received_ros_cloud = None
        self.pcd = None
        self.mono8_image = None
        self.merge_pcd = o3d.geometry.PointCloud()

    def run(self):
        while not rospy.is_shutdown():
            if (self.pcd is not None):
                pcd = copy.deepcopy(self.pcd)
                self.pcd = None
                pose_now = move_group.get_current_pose().pose
                base_endeff = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
                                                quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
                base2cam = np.matmul(base_endeff,self.eff_moveit2cam_moveit)
                # pcd.transform(np.linalg.inv(base2cam))
                pcd.transform(base2cam)
                self.merge_pcd += pcd
                self.merge_pcd.voxel_down_sample(voxel_size=0.001)

            break


    def get_mergpcd(self):
        return self.merge_pcd
        

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
        
class Hole_detector:
    def __init__(self):

        self.box = fixbox(Rz(-45),[1.005, 0.695, 0.55],0,x=0.2,y=0.3,z = 0.3)

    def Cluster_hole(self,pcd, Clus_eps=0.008, Clus_min_points=5, Out_nb_neighbors = 20, Out_std_ratio = 1.0, point_upper_limit = 100, point_lower_limit = 3, obj_size = 0.05):
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
        pcd = pcd_original.crop(self.box)
        pcd = pcd.voxel_down_sample(0.003)
        o3d.visualization.draw_geometries([pcd,Realcoor])

        pcdcen, ind = pcd.remove_statistical_outlier(nb_neighbors = 10,
                                                    std_ratio = 2.0)
        
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
        pcdcen = pcdcen.voxel_down_sample(0.001)


        # find boundarys in tensor
        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen, o3d.core.float64)
        pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)
        boundarys, mask = pcd_t.compute_boundary_points(0.01, 30)
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


rospy.init_node('merge_pcd', anonymous=True)


rate = rospy.Rate(10) # 10hz

# Setup Moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

move_group.set_max_velocity_scaling_factor(0.01)
move_group.set_max_acceleration_scaling_factor(0.01)


def control_cartesian_arm(position,rotation):
    rot = np.eye(4)
    rot[:3,:3] = rotation
    orientation = tf.transformations.quaternion_from_matrix(rot)

    waypoints = []
    pose_goal = move_group.get_current_pose().pose

    #### Single goal point

    pose_goal.position.x = position[0]
    pose_goal.position.y = position[1]
    pose_goal.position.z = position[2]

    pose_goal.orientation.x = orientation[0]
    pose_goal.orientation.y = orientation[1]
    pose_goal.orientation.z = orientation[2]
    pose_goal.orientation.w = orientation[3]

    waypoints.append(copy.deepcopy(pose_goal))

    (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.001, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

    # print(f"plan : {len(plan.joint_trajectory.points) }")

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
        # Publish
    display_trajectory_publisher.publish(display_trajectory)

    move_group.execute(plan, wait=True)
    rospy.sleep(0.1)


def remove_allscene():
    names = scene.get_objects()
    print(f"names : {names}")
    for name in names: 
        print(f"name : {name}")
        rospy.sleep(0.2)
        scene.remove_world_object(name)

def set_scene():

    rospy.sleep(0.5)
    ground_pose = geometry_msgs.msg.PoseStamped()
    ground_pose.header.frame_id = "base_link"
    ground_pose.pose.orientation.w = 1
    ground_pose.pose.position.x = 0
    ground_pose.pose.position.y = 0
    ground_pose.pose.position.z = -0.01
    scene.add_box('ground', ground_pose, size=(2,2,0.01))
    rospy.sleep(0.5)

    cam_pose = geometry_msgs.msg.PoseStamped()
    cam_pose.header.frame_id = "tool0_controller"
    cam_pose.pose.orientation.w = 1
    cam_pose.pose.position.x = 0
    cam_pose.pose.position.y = 0
    cam_pose.pose.position.z = 0.05
    scene.add_box('cam', cam_pose, size=(0.15,0.4,0.15))
    rospy.sleep(0.5)
    move_group.attach_object("cam", "tool0")
    # scene.attach_box("tool0","cam",touch_links = "tool0")
    rospy.sleep(1)

remove_allscene()
set_scene()
pose_goal = geometry_msgs.msg.Pose()

listener = tf.TransformListener()

scanner = hikrobot_merge()
img = np.ones((50,50))
Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
hole_detector = Hole_detector()
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

    if waitkeyboard & 0xFF==ord('p'):
        mergpcd = scanner.get_mergpcd()
        o3d.visualization.draw_geometries([mergpcd,Realcoor])
        # o3d.io.write_point_cloud(f"/home/oongking/Research_ws/src/hole_detector/mergepcd.pcd", mergpcd)
        center_point,plane_normal = hole_detector.find_hole(mergpcd)

        center_point = np.asarray(center_point)

        sign = np.sign(np.dot(center_point[0],plane_normal))
        if sign>0:
            plane_normal *= -1

        shift = np.multiply(plane_normal,0.6)

        shift = np.full([center_point.shape[0],3],shift)
        laser_points = np.add(center_point,shift)

        sort_i = np.argsort(laser_points[:,2])
        laser_points = laser_points[sort_i]

        off_set = []
        for point in laser_points:
            off_set.append(sphere(point).pcd)
        
        o3d.visualization.draw_geometries(off_set+[mergpcd,Realcoor])

        coors = []
        tfms = []
        
        for point in laser_points:
            laser_tf = np.eye(4)
            #offset
            laser_tf[:3,3] = point#+[0,0,0.0215] + np.array([0.707106,-0.707106,0])*0.004
            z_dir_rot = rotation_matrix_from_vectors([0,0,-1],plane_normal)[0]
            y = [0,1,0]
            new_y = np.matmul(z_dir_rot,y)
            y_dir_rot = rotation_matrix_from_vectors(new_y,[0,0,-1])[0]

            laser_tf[:3,:3] =  np.matmul(y_dir_rot,z_dir_rot)

            laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
            laser_coor.transform(laser_tf)
            coors.append(laser_coor)
            tfms.append(laser_tf)


        o3d.visualization.draw_geometries(off_set+coors+[mergpcd,Realcoor])

        laser2cam = np.linalg.inv(scanner.cam2laser)
        cam2eff = np.linalg.inv(scanner.eff_moveit2cam_moveit)
        laser2eff = np.matmul(laser2cam,cam2eff)

        for i,tfm in enumerate(tfms):

            tfm_go = np.matmul(tfm,laser2eff)
            control_cartesian_arm(tfm_go[:3,3],tfm_go[:3,:3])
            print("going")
            # o3d.visualization.draw_geometries(off_set+[mergpcd,coors[i],Realcoor])
            out = False
            while not rospy.is_shutdown():
                img = img = np.ones((100,100))
                cv2.imshow('img', img)
                waitkeyboard = cv2.waitKey(1)
                if waitkeyboard & 0xFF==ord('g'):
                    img = img = np.ones((50,50))
                    print("Next")
                    break
                if waitkeyboard & 0xFF==ord('q'):
                    print("finish")
                    out = True
                    break
            
            if out:
                break

        break

	
    

	

