#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
import math
import numpy as np

try:
    from urdf_parser_py.urdf import URDF
    from pykdl_utils.kdl_kinematics import KDLKinematics
except:
    rospy.logerr("Failed to import modules")


# HELPER FUNCTIONS

def rad_to_deg(rad):
    r = rad
    while (r < 0.0):
        r = r+(2.0*math.pi)

    while (r > 2.0*math.pi):
        r = r-(2.0*math.pi)

    degree = (r/math.pi)*180.0
    return degree

def convert_angles_deg(angles):
    new_angles = []
    for i in range(0, len(angles)):
        new_angles.append(rad_to_deg(angles[i]))
    return new_angles

def convert_angles_list(angles_list):
    new_angles = []
    for i in range(0, len(angles_list)):
        new_angles.append([])
        for j in range(0, len(angles_list[i])):
            new_angles[i].append(rad_to_deg(angles_list[i][j]))

    return new_angles


class KortexArm:
    def __init__(self):
        try:
            rospy.init_node('kortex_arm_controller')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_joint_speeds_full_name = '/' + self.robot_name + '/base/send_joint_speeds_command'
            rospy.wait_for_service(send_joint_speeds_full_name)
            self.send_joint_speeds_command = rospy.ServiceProxy(send_joint_speeds_full_name, SendJointSpeedsCommand)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            self.state_subscriber = rospy.Subscriber('/' + self.robot_name + '/joint_states', JointState, self.get_joint_state)
            self.state = JointState()
            self.joint_positions = [0.0]*self.degrees_of_freedom

            robot_description_full_name = self.robot_name + '/robot_description'
            robot_urdf = URDF.from_parameter_server(key=robot_description_full_name)
            self.kinematics_solver = KDLKinematics(robot_urdf, 'base_link', 'end_effector_link')

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def get_joint_state(self, data):
        self.state = data
        self.joint_positions = self.state.position[1:]

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def arm_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def send_home(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()


    def send_joint_angles(self, angles, output=True):
        self.last_action_notif_type = None
        req = PlayJointTrajectoryRequest()
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = float(angles[i])
            req.input.joint_angles.joint_angles.append(temp_angle)

        if output:
            rospy.loginfo("Sending the joint goal...")
        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()


    def send_joint_speeds(self, speeds, output=True):
        self.last_action_notif_type = None
        req = SendJointSpeedsCommandRequest()
        for i in range(self.degrees_of_freedom):
            temp_speed = JointSpeed()
            temp_speed.joint_identifier = i
            temp_speed.value = float(speeds[i])
            req.input.joint_speeds.append(temp_speed)

        if output:
            rospy.loginfo("Sending the joint speeds...")
        try:
            self.send_joint_speeds_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendJointSpeedsCommand")
            return False
        else:
            return True

    def send_gripper_value(self, value, output=True):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        if output:
            rospy.loginfo("Sending the gripper command...")
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            return True

    def stop(self, output=True):
        if(self.send_joint_speeds(self.degrees_of_freedom*[0.0], output=output)):
            if output:
                rospy.loginfo("Successfully stopped Kortex Arm")
            return True
        else:
            rospy.logerr("Failed to stop Kortex Arm")
            return False

    def send_eef_position(self, x_desired, output=True):
        success = False
        x_d = np.matrix(x_desired).T
        x_a = np.matrix(self.__end_effector_location()).T
        d_x = np.matrix(np.zeros(len(x_d))).T
        theta_a = np.matrix(self.joint_positions).T
        d_theta = np.matrix(np.ones(self.degrees_of_freedom)).T
        while(np.linalg.norm(d_theta) > 0.00005):
            d_x = x_d-x_a
            jac = self.kinematics_solver.jacobian(theta_a)
            # We are not considering orientation here
            jac = np.delete(jac, [3,4,5], 0)
            d_theta = 0.1*(self.__dls(jac, d_x, 0.5))
            theta_a = np.add(theta_a, d_theta)
            x_a = np.matrix(self.__end_effector_location(angles=theta_a.T.tolist()[0])).T
            if(np.linalg.norm(x_a-x_d) < 0.001):
                success = True
                break
        if success:
            self.send_joint_angles(convert_angles_deg(theta_a.T.tolist()[0]), output=output)
        else:
            rospy.logwarn("Unable to calculate theta value for desired end-effector location")
        return success

    def send_eef_velocity(self, x_dot, duration=1.0, output=True):
        success = False
        duration = float(duration)
        runtime = 0.0
        x_dot = np.matrix(x_dot).T
        d_theta = np.matrix(np.ones(self.degrees_of_freedom)).T
        last_eef_pos = self.__end_effector_location()
        dt = (1.0/10.0)
        kp = 1
        start_loop = time.time()
        last_time = time.time()
        acc = 0.0
        try:
            if output:
                rospy.loginfo("Attempting requested end-effector velocity...")
            while(np.linalg.norm(d_theta) > 0.005):
                next_time = last_time - time.time()
                acc += next_time
                last_time = time.time()
                if(acc >= dt):
                    eef_pos = self.__end_effector_location()
                    proj_eef_pos = last_eef_pos + dt*x_dot
                    E = proj_eef_pos-eef_pos
                    current_time = time.time()
                    jac = self.kinematics_solver.jacobian(self.joint_positions)
                    # We are not considering orientation here
                    jac = np.delete(jac, [3,4,5], 0)
                    feedback_x_dot = x_dot+kp*E
                    d_theta = self.__dls(jac, feedback_x_dot, 0.5)
                    self.send_joint_speeds(convert_angles_deg(d_theta.T.tolist()[0]), output=False)
                    runtime += time.time()-start_loop
                    start_loop = time.time()
                    if(runtime > duration):
                        success = True
                        break
                    acc-=dt
                    last_eef_pos = self.__end_effector_location()
        except KeyboardInterrupt:
            rospy.logwarn("End-effector motion discontinued")
        self.stop(output=False)
        if success and output:
            rospy.loginfo("Successfully executed end-effector motion for desired duration")
        else:
            rospy.logwarn("Failed to move end-effector at desired velocity for full duration")
        return success

    def startup(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            success &= self.arm_clear_faults()
            success &= self.subscribe_to_a_robot_notification()
            success &= self.send_home()
            if self.is_gripper_present:
                success &= self.send_gripper_value(0.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")

        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)
        if not success:
            rospy.logerr("The example encountered an error.")

    def __end_effector_location(self, angles=None):
        if angles is None:
            angles = self.joint_positions
        eef_fk_mat = self.kinematics_solver.forward(angles)
        return np.array([eef_fk_mat[0,3], eef_fk_mat[1,3], eef_fk_mat[2,3]])

    def __dls(self, jac, d_x, damp_factor):
        m = jac.shape[0]
        return np.matmul(np.matmul(jac.T, np.linalg.inv(np.matmul(jac, jac.T) + (damp_factor**2)*np.identity(m))), d_x)
