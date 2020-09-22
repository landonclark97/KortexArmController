#!/usr/bin/env python

# Imports
import sys
import copy
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from control_msgs.msg import JointControllerState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
import scipy.io as scio
import math
from tf.transformations import quaternion_from_matrix
import numpy as np
import time
import threading

# FK Imports
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics


# GLOBALS ########################################################
ERROR_THRESHOLD = 0.01
moving = []

waypoint_mat = scio.loadmat('/home/landon/src/kortex_sim/trajectory_direct.mat')
waypoint_arr = waypoint_mat['trajectory_direct']

eef_waypoints = []
curr_angles = []
##################################################################



# Computing Forward Kinematics ###################################
robot_urdf = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot_urdf, robot_urdf.links[1].name, robot_urdf.links[9].name)


for i in range(0, len(waypoint_arr)):

    waypoint_arr[i][0] = -waypoint_arr[i][0]
    waypoint_arr[i][2] = -waypoint_arr[i][2]
    waypoint_arr[i][4] = -waypoint_arr[i][4]
    waypoint_arr[i][6] = -waypoint_arr[i][6]


    fk_mat = kdl_kin.forward(waypoint_arr[i])
    quat = quaternion_from_matrix(fk_mat)
    pose_goal = Pose(Point(fk_mat[0,3],fk_mat[1,3],fk_mat[2,3]), Quaternion(quat[0], quat[1], quat[2], quat[3]))

    eef_waypoints.append(copy.deepcopy(pose_goal))
##################################################################



def handle_joint_pos():

    dof = 7
    pub = []
    sub = []

    for i in range(1,dof+1):
        pub.append(rospy.Publisher('/my_gen3/joint_' + str(i) + '_position_controller/command', Float64, queue_size=10))
        moving.append(True)
        curr_angles.append(0)

    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    sub.append(rospy.Subscriber('/my_gen3/joint_1_position_controller/state', JointControllerState, callback1))
    sub.append(rospy.Subscriber('/my_gen3/joint_2_position_controller/state', JointControllerState, callback2))
    sub.append(rospy.Subscriber('/my_gen3/joint_3_position_controller/state', JointControllerState, callback3))
    sub.append(rospy.Subscriber('/my_gen3/joint_4_position_controller/state', JointControllerState, callback4))
    sub.append(rospy.Subscriber('/my_gen3/joint_5_position_controller/state', JointControllerState, callback5))
    sub.append(rospy.Subscriber('/my_gen3/joint_6_position_controller/state', JointControllerState, callback6))
    sub.append(rospy.Subscriber('/my_gen3/joint_7_position_controller/state', JointControllerState, callback7))

    rate = rospy.Rate(50) # 50hz


    for i in range(0, len(waypoint_arr)):
        for j in range(0, dof):
            pub[j].publish(waypoint_arr[i][j])
            moving[j] = True

        plot_desired_eef(marker_publisher, i)

        '''
        while True:
            if not arm_moving():
                break
            time.sleep(0.0001)
        '''

        rate.sleep()

        plot_actual_eef(marker_publisher, i+len(waypoint_arr))



def plot_desired_eef(pub, i):
    marker = Marker(
                type=Marker.SPHERE,
                id=i,
                lifetime=rospy.Duration(1000),
                pose=eef_waypoints[i],
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id="base_link"),
                color=ColorRGBA(0.0, 2.0, 0.0, 0.8))

    pub.publish(marker)


def plot_actual_eef(pub, i):

    fk_mat = kdl_kin.forward(curr_angles)

    quat = quaternion_from_matrix(fk_mat)
    pose_goal = Pose(Point(fk_mat[0,3],fk_mat[1,3],fk_mat[2,3]), Quaternion(quat[0], quat[1], quat[2], quat[3]))

    marker = Marker(
                type=Marker.SPHERE,
                id=i,
                lifetime=rospy.Duration(1000),
                pose=pose_goal,
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id="base_link"),
                color=ColorRGBA(0.0, 0.0, 2.0, 0.8))

    pub.publish(marker)



def arm_moving():
    for i in range(0, len(moving)):
        if moving[i]:
            return True
    return False



def callback1(data):
    curr_angles[0] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[0] = False

def callback2(data):
    curr_angles[1] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[1] = False

def callback3(data):
    curr_angles[2] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[2] = False

def callback4(data):
    curr_angles[3] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[3] = False

def callback5(data):
    curr_angles[4] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[4] = False

def callback6(data):
    curr_angles[5] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[5] = False

def callback7(data):
    curr_angles[6] = data.process_value
    if abs(data.error) < ERROR_THRESHOLD:
        moving[6] = False




if __name__ == '__main__':
    try:

        rospy.loginfo("Unpausing")
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        resp = unpause_gazebo()
        rospy.loginfo("Unpaused")


        rospy.init_node('control_trajectory', anonymous=True)
        handle_joint_pos()

    except rospy.ROSInterruptException:
        pass
