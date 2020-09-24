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


MOVEMENT_RES = 0.01


########################################
## QUATERNION INTERPOLATION FUNC #######
########################################

# http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
def slerp(qa, qb, t):
    # quaternion to return
    qm = quaternion_about_axis(0, (1, 0, 0))
    # Calculate angle between them.
    cosHalfTheta = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]
    if (cosHalfTheta < 0):
        qb[0] = -qb[0]
        qb[1] = -qb[1]
        qb[2] = -qb[2]
        qb[3] = qb[3]
    cosHalfTheta = -cosHalfTheta
    # if qa=qb or qa=-qb then theta = 0 and we can return qa
    if (abs(cosHalfTheta) >= 1.0):
        qm[0] = qa[0]
        qm[1] = qa[1]
        qm[2] = qa[2]
        qm[3] = qa[3]
        return qm
    # Calculate temporary values.
    halfTheta = math.acos(cosHalfTheta)
    sinHalfTheta = math.sqrt(1.0 - cosHalfTheta*cosHalfTheta)
    # if theta = 180 degrees then result is not fully defined
    # we could rotate around any axis normal to qa or qb
    if (abs(sinHalfTheta) < 0.001):
        qm[0] = (qa[0] * 0.5 + qb[0] * 0.5)
        qm[1] = (qa[1] * 0.5 + qb[1] * 0.5)
        qm[2] = (qa[2] * 0.5 + qb[2] * 0.5)
        qm[3] = (qa[3] * 0.5 + qb[3] * 0.5)
        return qm
    ratioA = math.sin((1 - t) * halfTheta) / sinHalfTheta
    ratioB = math.sin(t * halfTheta) / sinHalfTheta
    # calculate Quaternion.
    qm[0] = (qa[0] * ratioA + qb[0] * ratioB)
    qm[1] = (qa[1] * ratioA + qb[1] * ratioB)
    qm[2] = (qa[2] * ratioA + qb[2] * ratioB)
    qm[3] = (qa[3] * ratioA + qb[3] * ratioB)
    return qm

########################################
########################################
########################################





def steps_to_goal(pos1, pos2):
    norm_pos1 = math.sqrt((pos1[0]*pos1[0])+(pos1[1]*pos1[1])+(pos1[2]*pos1[2]))
    norm_pos2 = math.sqrt((pos2[0]*pos2[0])+(pos2[1]*pos2[1])+(pos2[2]*pos2[2]))

    return math.ceil(abs(norm_pos1-norm_pos2)/MOVEMENT_RES)


def percent_change(x, y, percent):
    diff = x-y
    return x-(diff*percent)


def interpolate_position(pos1, pos2, steps, t):
    curr_percent = (t/steps)

    new_x = percent_change(pos1[0], pos2[0], curr_percent)
    new_y = percent_change(pos1[1], pos2[1], curr_percent)
    new_z = percent_change(pos1[2], pos2[2], curr_percent)

    return [new_x, new_y, new_z]




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

while True:
    in_px = input("input x coordinate of object\n")
    in_px = float(in_px)
    in_py = input("input y coordinate of object\n")
    in_py = float(in_py)
    in_pz = input("input z coordinate of object\n")
    in_pz = float(in_pz)

    in_pxyz_norm = math.sqrt((in_px*in_px)+(in_py*in_py)+(in_pz*in_pz))
    if in_pxyz_norm <= 0.95 and in_pz > 0:
        break
    else:
        print "value out of reach, input new coordinates"


in_theta = input("input theta (radians)\n")
in_theta = float(in_theta)
in_nx = input("input axis n_x\n")
in_nx = float(in_nx)
in_ny = input("input axis n_y\n")
in_ny = float(in_ny)
in_nz = input("input axis n_z\n")
in_nz = float(in_nz)

in_nxyz_norm = math.sqrt((in_nx*in_nx)+(in_ny*in_ny)+(in_nz*in_nz))

in_nx = in_nx/in_nxyz_norm
in_ny = in_ny/in_nxyz_norm
in_nz = in_nz/in_nxyz_norm

in_q = quaternion_about_axis(in_theta, (in_nx, in_ny, in_nz))




while True:
    final_px = input("target x\n")
    final_px = float(final_px)
    final_py = input("target y\n")
    final_py = float(final_py)
    final_pz = input("target z\n")
    final_pz = float(final_pz)

    final_pxyz_norm = math.sqrt((final_px*final_px) + (final_py*final_py)+(final_pz*final_pz))
    if final_pxyz_norm <= 0.95 and final_pz > 0:
        break
    else:
        print "value out of reach, input new coordinates"

final_theta = input("target theta (radians)\n")
final_theta = float(final_theta)
final_nx = input("target axis n_x\n")
final_nx = float(final_nx)
final_ny = input("target axis n_y\n")
final_ny = float(final_ny)
final_nz = input("target axis n_z\n")
final_nz = float(final_nz)

final_nxyz_norm = math.sqrt((final_nx*final_nx)+(final_ny*final_ny)+(final_nz*final_nz))

final_nx = final_nx/final_nxyz_norm
final_ny = final_ny/final_nxyz_norm
final_nz = final_nz/final_nxyz_norm

final_q = quaternion_about_axis(final_theta, (final_nx, final_ny, final_nz))



########################################
########################################
########################################




start_pose = group.get_current_pose().pose
start_quat = [start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z]
start_pos = [start_pose.position.x, start_pose.position.y, start_pose.position.z]

in_pos = [in_px, in_py, in_pz]
final_pos = [final_px, final_py, final_pz]

steps = steps_to_goal(start_pos, in_pos)
steps = int(steps)

print "steps: ", steps

pose_goal = Pose()

waypoints = []
waypoints.append(copy.deepcopy(start_pose))

for i in range(1, steps+1):


    temp_quat = slerp(start_quat, in_q, i/steps)
    temp_pos = interpolate_position(start_pos, in_pos, steps, i)


    pose_goal.position.x = temp_pos[0]
    pose_goal.position.y = temp_pos[1]
    pose_goal.position.z = temp_pos[2]

    pose_goal.orientation.w = temp_quat[0]
    pose_goal.orientation.x = temp_quat[1]
    pose_goal.orientation.y = temp_quat[2]
    pose_goal.orientation.z = temp_quat[3]


    waypoints.append(copy.deepcopy(pose_goal))



(plan, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.0001,        # eef_step
                                0.0)         # jump_threshold


display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)


display_trajectory_publisher.publish(display_trajectory)
group.execute(plan, wait=True)





steps = steps_to_goal(in_pos, final_pos)
steps = int(steps)

pose_goal = Pose()


pose_goal.position.x = in_pos[0]
pose_goal.position.y = in_pos[1]
pose_goal.position.z = in_pos[2]

pose_goal.orientation.w = in_q[0]
pose_goal.orientation.x = in_q[1]
pose_goal.orientation.y = in_q[2]
pose_goal.orientation.z = in_q[3]


waypoints = []
waypoints.append(copy.deepcopy(pose_goal))

for i in range(1, steps+1):


    temp_quat = slerp(in_q, final_q, i/steps)
    temp_pos = interpolate_position(in_pos, final_pos, steps, i)


    pose_goal.position.x = temp_pos[0]
    pose_goal.position.y = temp_pos[1]
    pose_goal.position.z = temp_pos[2]

    pose_goal.orientation.w = temp_quat[0]
    pose_goal.orientation.x = temp_quat[1]
    pose_goal.orientation.y = temp_quat[2]
    pose_goal.orientation.z = temp_quat[3]


    waypoints.append(copy.deepcopy(pose_goal))



(plan, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.0001,        # eef_step
                                0.0)         # jump_threshold


display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)


display_trajectory_publisher.publish(display_trajectory)
group.execute(plan, wait=True)

