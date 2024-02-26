#!/usr/bin/env python

# Ros
import rospy
import rosnode
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


class hikrobot_merge():
    def __init__(self):

        rospy.loginfo(":: Starting hik ::")

        rospy.Subscriber("/hik/image_raw", Image, self.mono8_callback)
        # rospy.Subscriber("/hik/pointcloud2", PointCloud2, self.points_callback)  

        self.eff_moveit2cam_moveit = np.array([[ 0.00550817,  0.28519852,  0.95845264,  0.29277144],
                                                [-0.99994433, -0.00705541,  0.00784604,  0.0032999 ],
                                                [ 0.00899996, -0.9584425,   0.28514377, -0.09824983],
                                                [ 0.,          0.,          0.,          1.,       ]])

        self.received_ros_cloud = None
        self.pcd = None
        self.mono8_image = None
        self.merge_pcd = o3d.geometry.PointCloud()

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
        
def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

rospy.init_node("hik_imgcap", anonymous=True)
cam = hikrobot_merge()
i = 5
while not rospy.is_shutdown():

    mono_img = cam.get_img()
    cv2.imshow("mono_img",resize_percent(mono_img,50))

    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('s'):
        cv2.imwrite(f"/home/oongking/Research_ws/src/hole_detector/data/hikrobot/laser/{i}.png",mono_img)
        i+=1
        cv2.destroyAllWindows()
        rospy.sleep(1)

    if waitkeyboard & 0xFF==ord('q'):
        print("===== End =====")
        cv2.destroyAllWindows()
        break


cv2.destroyAllWindows()