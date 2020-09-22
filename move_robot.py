#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import scipy.io as scio
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import time


from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

# pykdl_utils setup
robot_urdf = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot_urdf, "base_link", "end_effector_link")



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
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


pose_goal = geometry_msgs.msg.Pose()
print "\n\n\n===== early pose: %s" % pose_goal

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


mat = scio.loadmat('/home/landon/src/kortex_sim/trajectory_direct.mat')
move_angles = mat['trajectory_direct']

joint_goal = group.get_current_joint_values()

'''

joint_goal[0] = move_angles[0][0]
joint_goal[1] = move_angles[0][1]
joint_goal[2] = move_angles[0][2]
joint_goal[3] = move_angles[0][3]
joint_goal[4] = move_angles[0][4]
joint_goal[5] = move_angles[0][5]
joint_goal[6] = move_angles[0][6]

group.go(joint_goal, wait=True)

time.sleep(10)

move_angles = move_angles[1:]

waypoints = []

for i in range(0, len(move_angles)):
    fk_mat = kdl_kin.forward(move_angles[i])

    print "==== fk_mat: %s" % fk_mat

    pose_goal.position.x = fk_mat[0,3]
    pose_goal.position.y = fk_mat[1,3]
    pose_goal.position.z = fk_mat[2,3]

    w = math.sqrt(1+fk_mat[0,0]+fk_mat[1,1]+fk_mat[2,2])/2
    x = (fk_mat[2,1]-fk_mat[1,2])/(4*w)
    y = (fk_mat[0,2]-fk_mat[2,0])/(4*w)
    z = (fk_mat[1,0]-fk_mat[0,1])/(4*w)

    pose_goal.orientation.w = w
    pose_goal.orientation.x = x
    pose_goal.orientation.y = y
    pose_goal.orientation.z = z

    waypoints.append(copy.deepcopy(pose_goal))

fraction = 0.0


while fraction < 0.10:

    try:
        (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.1,        # eef_step
                                   0.0)         # jump_threshold
        print "current fraction: ", fraction

    except KeyboardInterrupt:
        break

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)


display_trajectory_publisher.publish(display_trajectory)
group.execute(plan, wait=True)

'''
for i in range(0, len(move_angles)):

    joint_goal[0] = move_angles[i][0]
    joint_goal[1] = move_angles[i][1]
    joint_goal[2] = move_angles[i][2]
    joint_goal[3] = move_angles[i][3]
    joint_goal[4] = move_angles[i][4]
    joint_goal[5] = move_angles[i][5]
    joint_goal[6] = move_angles[i][6]

    group.go(joint_goal, wait=True)
