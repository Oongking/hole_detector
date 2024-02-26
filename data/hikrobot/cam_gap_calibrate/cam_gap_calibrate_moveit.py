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

class hikrobot_merge():
    def __init__(self):

        rospy.loginfo(":: Starting hik ::")

        rospy.Subscriber("/hik/image_raw", Image, self.mono8_callback)
        rospy.Subscriber("/hik/pointcloud2", PointCloud2, self.points_callback)  

        self.eff_moveit2cam_moveit = np.array([[ 0.00550817,  0.28519852,  0.95845264,  0.29277144],
                                                [-0.99994433, -0.00705541,  0.00784604,  0.0032999 ],
                                                [ 0.00899996, -0.9584425,   0.28514377, -0.09824983],
                                                [ 0.,          0.,          0.,          1.,       ]])

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

    def get_img(self):
        
        while 1:
            if (self.mono8_image is not None):
                break
        mono8_image = copy.deepcopy(self.mono8_image)

        self.mono8_image = None
        return mono8_image
        

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
        
def tf_rot_trans(tfms):
    rots = []
    trans =[]

    for tfm in tfms:
        rots.append(tfm[:3,:3])
        trans.append(tfm[:3,3])

    return rots,trans

rospy.init_node('find_cam_gap', anonymous=True)

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

hik = hikrobot_merge()
arm_TFMs = []
ar_TFMs = []
num = 0
while not rospy.is_shutdown():

    img = hik.get_img()
    cv2.imshow('img', resize_percent(img,50))
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('a'):
        
        (corners, ids,rejected)= cv2.aruco.detectMarkers(img,arucoDictA3,parameters=arucoParams)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(img,corners,ids)
            rvec=None
            tvec=None
            _,rvec,tvec = cv2.aruco.estimatePoseBoard( corners, ids, zivid_boardA3, hikrobot_camera_matrix, hikrobot_distortion_coefficients,rvec,tvec)

            if rvec is not None and tvec is not None:
                R, _ = cv2.Rodrigues(rvec)
                ar_tf = np.eye(4)
                ar_tf[:3, :3] = R
                ar_tf[:3, 3] = tvec.reshape(-1)
                ar_TFMs.append(ar_tf)
                pose_now = move_group.get_current_pose().pose
                arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
                                                quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
                arm_TFMs.append(arm_TFM)
                
                # print("arm_TFM : ",np.asarray(arm_TFM))
                num +=1
                print(f"num = : {num}")
                # cv2.aruco.drawAxis( img, hikrobot_camera_matrix, hikrobot_distortion_coefficients, rvec, tvec, 0.08 )
                # while True:
                #     cv2.imshow('img', resize_percent(img,50))
                #     if cv2.waitKey(1) & 0xFF==ord('q'):
                #         Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.3,(0,0,0))
                #         ar_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
                #         R, _ = cv2.Rodrigues(rvec)
                #         ar_tf = np.eye(4)
                #         ar_tf[:3, :3] = R
                #         ar_tf[:3, 3] = tvec.reshape(-1)
                #         ar_coor.transform(ar_tf)
                #         o3d.visualization.draw_geometries([Realcoor,ar_coor])
                #         break
        
        rospy.sleep(1)

    
    if waitkeyboard & 0xFF==ord('q'):
        break


arm_rot,arm_trans = tf_rot_trans(arm_TFMs)
ar_rot,ar_trans = tf_rot_trans(ar_TFMs)

R_off, T_off = cv2.calibrateHandEye(arm_rot,arm_trans,ar_rot,ar_trans,cv2.CALIB_HAND_EYE_TSAI)

endeff2cam = np.eye(4)
endeff2cam[:3, :3] = R_off
endeff2cam[:3, 3] = T_off.reshape(-1)


Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.3,(0,0,0))
eff_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))

pose_now = move_group.get_current_pose().pose
arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
                                quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
eff_coor.transform(arm_TFM)
cam_coor.transform(np.matmul(arm_TFM,endeff2cam))

print(f"endeff2cam : {endeff2cam}")
o3d.visualization.draw_geometries([Realcoor,eff_coor,cam_coor])


