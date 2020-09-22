#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import scipy.io as scio
import math
from tf.transformations import quaternion_from_matrix, quaternion_about_axis, quaternion_multiply
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import time

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics


# pykdl_utils setup
robot_urdf = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot_urdf, "base_link", "end_effector_link")




########################################
## INITIALIZE MOVEIT ###################
########################################


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_to_loc',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

group.allow_replanning(True)

# Allow some leeway in position(meters) and orientation (radians)
group.set_goal_position_tolerance(0.01)
group.set_goal_orientation_tolerance(0.1)


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


########################################
########################################
########################################




########################################
## HANDLE INPUT ########################
########################################

in_x = input("input x coordinate of object")
in_x = float(in_x)
in_y = input("input y coordinate of object")
in_y = float(in_y)
in_z = input("object height")
in_z = float(in_z)

final_x = input("target x")
final_x = float(final_x)
final_y = input("target y")
final_y = float(final_y)

final_z = 0
while(final_z < in_z):
    final_z = input("target z")
    final_z = float(final_z)


########################################
########################################
########################################


q = quaternion_about_axis(math.pi, (0, 1, 0))

initial_pose = Pose()

initial_pose.position.x = in_x
initial_pose.position.y = in_y
initial_pose.position.z = in_z
initial_pose.orientation.w = q[0]
initial_pose.orientation.x = q[1]
initial_pose.orientation.y = q[2]
initial_pose.orientation.z = q[3]

xy_norm = math.sqrt((final_x*final_x) + (final_y*final_y))

normal_x = final_x/xy_norm
normal_y = final_y/xy_norm

z_rot = math.atan2(normal_y, normal_x)

q1 = quaternion_about_axis(z_rot, (0, 0, 1))
q2 = quaternion_about_axis((math.pi/2), (0, 1, 0))

q_final = quaternion_multiply(q1, q2)

final_pose = Pose()
final_pose.position.x = final_x
final_pose.position.y = final_y
final_pose.position.z = final_z
final_pose.orientation.w = q_final[0]
final_pose.orientation.x = q_final[1]
final_pose.orientation.y = q_final[2]
final_pose.orientation.z = q_final[3]










########################################
## PRINT ###############################
########################################


# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

########################################
########################################
########################################











group.set_pose_target(initial_pose)
plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

time.sleep(2)

group.set_pose_target(final_pose)
plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
