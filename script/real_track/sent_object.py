#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry

# Image
import cv2

# 3D
import open3d as o3d

# Utility
# from utils import *
import getch
import numpy as np
import time
import copy
import sys
import geometry_msgs.msg


rospy.init_node('test_object', anonymous=True)


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

remove_allscene()


def allow_collisions_with_object(obj_name, scene):
    """ Updates the MoveIt PlanningScene using the AllowedCollisionMatrix to ignore collisions for an object """
    # Set up service to get the current planning scene
    service_timeout = 5.0
    _get_planning_scene = rospy.ServiceProxy(
        "get_planning_scene", GetPlanningScene
    )
    _get_planning_scene.wait_for_service(service_timeout)

    request = GetPlanningScene()
    request.components = 0    # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)
    print(f"\n\n\n--- allowed_collision_matrix before update:{planning_scene.scene.allowed_collision_matrix} ---\n\n\n")

    # Set this object to ignore collisions with all objects. The entry values are not updated
    planning_scene.scene.allowed_collision_matrix.entry_names.append(obj_name)
    for entry in planning_scene.scene.allowed_collision_matrix.entry_values:
        entry.enabled.append(False)
    enabled = [False for i in range(len(planning_scene.scene.allowed_collision_matrix.entry_names))]
    entry = AllowedCollisionEntry(enabled=enabled)  # Test kwarg in constructor
    planning_scene.scene.allowed_collision_matrix.entry_values.append(entry)

    # Set the default entries. They are also not updated
    planning_scene.scene.allowed_collision_matrix.default_entry_names = [obj_name]
    planning_scene.scene.allowed_collision_matrix.default_entry_values = [False]
    planning_scene.scene.is_diff = True # Mark this as a diff message to force an update of the allowed collision matrix
    planning_scene.scene.robot_state.is_diff = True

    planning_scene_diff_req = ApplyPlanningSceneRequest()
    planning_scene_diff_req.scene = planning_scene.scene

    # Updating the Allowed Collision Matrix through the apply_planning_scene service shows no effect.
    # However, adding objects to the planning scene works fine.
    # scene._apply_planning_scene_diff.call(planning_scene_diff_req)
    scene.apply_planning_scene(planning_scene.scene)

    # Attempting to use the planning_scene topic for asynchronous updates also does not work
    # planning_scene_pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=5)
    # planning_scene_pub.publish(planning_scene.scene)

    print(f"\n--- Sent message:{planning_scene.scene.allowed_collision_matrix} ---\n")

    # The planning scene retrieved after the update should have taken place shows the Allowed Collision Matrix is the same as before
    request = GetPlanningScene()
    request.components = 0    # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)
    print(f"\n--- allowed_collision_matrix after update:{planning_scene.scene.allowed_collision_matrix} ---\n")



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

# allow_collisions_with_object("cam",scene)
# allow_collisions_with_object("ground",scene)