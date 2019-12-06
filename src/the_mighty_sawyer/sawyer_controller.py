#!/usr/bin/env python

"""
This file acts as a wrapper for the intera interface to control the sawyer arm.
It contains a class with various methods for a controlling the sawyer arm to
throw a bean bag.
"""

from __future__ import division
import numpy as np
import rospy
import yaml

import intera_interface
from intera_interface import CHECK_VERSION

import geometry_msgs.msg
from std_msgs.msg import String

class MoveArm(object):

    def __init__(self):
        """
        Class containing methods to fully interact with the sawyer arm.
        """
        # Import Script to Enable/Disable
        rs = intera_interface.RobotEnable(CHECK_VERSION)

        self.gripper = intera_interface.Gripper('right_gripper')

        self.InitializeGripper()

        self.limb = intera_interface.Limb()

        self.joint_names = self.limb.joint_names()

        # Make sure to import yaml file in launch file
    #     self._receive_home, self._throw_home = self.get_throw_params()

    def get_throw_params(self):
        with open('joints_cfg_sawyer.yaml') as f:
            # use safe_load instead load
            dataMap = yaml.safe_load(f)
            

    def InitializeGripper(self):
        self.gripper.reboot()
        self.gripper.calibrate()

    def OpenGripper(self):
        self.gripper.open()

    def CloseGripper(self):
        self.gripper.close()

    def go_to_recieving_pos(self):
        """
        Moves the arm to a preconfigured home position by setting joint
        angles directly.
        """
        print "Going to Home..."

        joint_goal = {'right_j0':0,
                      'right_j1':-0.5,
                      'right_j2':0,
                      'right_j3':2.5,
                      'right_j4':0,
                      'right_j5':-2}

        # self.limb.set_joint_position_speed(speed=0.3)
        self.limb.set_joint_positions(joint_goal)

    def print_joint_states(self):
        """
        Prints the joint angle for all joints
        """
        rospy.loginfo("Joint Values ===========================")
        rospy.loginfo(self.limb.joint_angles())

    def print_pose(self):
        """
        Prints the pose of the end effector
        """
        rospy.loginfo("End Effector Pose ============================")
        rospy.loginfo(self.limb.endpoint_pose())

    def calc_throw_start_pos(self):
        pass

    def go_to_throwing_start(self):
        """
        Moves the arm to a calculated position to begin throwing the bag.
        """
        cur_joint_angles = self.limb.joint_angles()

        joint_goal = {'right_j0':-1.5,
                      'right_j1':0,
                      'right_j2':cur_joint_angles[self.joint_names[2]],
                      'right_j3':1.5,
                      'right_j4':-1.5,
                      'right_j5':0}

        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.set_joint_positions(joint_goal)

        rospy.sleep(1)

        joint_goal = {'right_j2':-1.5}

        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.set_joint_positions(joint_goal)

    def throw(self):
        """
        Throws the bag by rotating
        """
        cur_joint_angles = self.limb.joint_angles()

        joint_goal = {'right_j2':1.5}

        self.limb.set_joint_position_speed(speed=1)
        self.limb.set_joint_positions(joint_goal)

        release = 0

        while release == 0:

            cur_joint_angles = self.limb.joint_angles()

            if cur_joint_angles[self.joint_names[2]] >= .2:
                rospy.loginfo("Open Grippers")
                self.OpenGripper()
                release = 1
