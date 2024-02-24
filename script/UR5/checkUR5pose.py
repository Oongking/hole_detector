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

def control_plannar_arm(pose):

	waypoints = []
	pose_goal = move_group.get_current_pose().pose

	#### Single goal point
	pose_goal.position.x = pose[0]
	pose_goal.position.y = pose[1]
	pose_goal.position.z = pose[2]
	pose_goal.orientation.x = pose[3]
	pose_goal.orientation.y = pose[4]
	pose_goal.orientation.z = pose[5]
	pose_goal.orientation.w = pose[6]

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
	rate.sleep()



rospy.init_node('ur5posecheck', anonymous=True)
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

pose_goal = geometry_msgs.msg.Pose()

listener = tf.TransformListener()

while not rospy.is_shutdown():

	# print(move_group.get_current_pose().pose)

	print("\n\n############# GET VALUE #############")

	key = getch.getch().lower()

	print("key : ",key)

	if key == 'j':
		joint_goal = move_group.get_current_joint_values()
		print(f" joint_goal : ",joint_goal)

	if key == 'c':
		pose_now = move_group.get_current_pose().pose
		arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
																						quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
		print("arm_TFM : ",np.asarray(arm_TFM))

	if key == 'p':
		print(f" Preview ")

		pose_now = move_group.get_current_pose().pose
		arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
																						quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
		print("arm_TFM : ",np.asarray(arm_TFM))

		Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.5,(0,0,0))

		Robotcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
		Robotcoor.transform(arm_TFM)


		o3d.visualization.draw_geometries([Realcoor,Robotcoor])


	rate.sleep()

# def checkUR5():
    
# if __name__ == '__main__':
#     try:
#         checkUR5()
#     except rospy.ROSInterruptException:
#         pass