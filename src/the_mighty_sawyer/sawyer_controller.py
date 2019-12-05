#!/usr/bin/env python

"""
This file acts as a wrapper for the intera interface to control the sawyer arm.
It contains a class with various methods for a controlling the sawyer arm to
throw a bean bag.
"""

from __future__ import division
import numpy as np
import rospy

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

    def InitializeGripper(self):
        self.gripper.reboot()
        rospy.sleep(3)
        self.gripper.calibrate()
        rospy.sleep(3)

    def OpenGripper(self):
        rospy.loginfo("Gripper Open")
        self.gripper.open()

    def CloseGripper(self):
        rospy.loginfo("Gripper Close")
        self.gripper.close()

    def go_to_recieving_pos(self):
        """
        Moves the arm to a preconfigured home position by setting joint
        angles directly.
        """
        print "Going to Home..."

        joint_goal = {'right_j0':0.003546875,
                      'right_j1':-0.5124091796875,
                      'right_j2':0.0377041015625,
                      'right_j3':1.8277978515625,
                      'right_j4':-0.048537109375,
                      'right_j5':-2.8403095703125,
                      'right_j6':-1.39600390625}

        self.limb.set_joint_position_speed(speed=0.3)

        rospy.loginfo(joint_goal)

        self.limb.move_to_joint_positions(joint_goal)

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

        rospy.loginfo("Start Throw")

        cur_joint_angles = self.limb.joint_angles()

        joint_goal = {'right_j0':-1.5,
                      'right_j1':0.0190947265625,
                      'right_j2':cur_joint_angles[self.joint_names[2]],
                      'right_j3':1.5639794921875,
                      'right_j4':-1.60975390625,
                      'right_j5':2.5}

        # joint_goal = {'right_j0':-1.5,
        #               'right_j1':0.0190947265625,
        #               'right_j2':cur_joint_angles[self.joint_names[2]],
        #               'right_j3':1.5639794921875,
        #               'right_j4':-1.60975390625,
        #               'right_j5':1}


        name: [head_pan, right_j0, right_j1, right_j2, right_j3, right_j4, right_j5, right_j6, torso_t0]
        position: [-1.0011552734375, -3.0510107421875, 0.0096748046875, -3.0412255859375, -0.0109296875, -0.074908203125, -0.1154091796875, -1.2699619140625, 0.0]

        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.move_to_joint_positions(joint_goal)

        rospy.sleep(.5)

        joint_goal = {'right_j2':-1.5}

        # joint_goal = {'right_j2':-.5}

        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.move_to_joint_positions(joint_goal)

    def throw(self):
        """
        Throws the bag by rotating
        """

        rospy.loginfo("Throwing...")

        cur_joint_angles = self.limb.joint_angles()

        joint_goal = {'right_j2':2,
                      'right_j5':-.75}

        joint_vels = {'right_j2':10,
                      'right_j5':10}

        self.limb.set_joint_position_speed(speed=.8)

        release = 0

        while cur_joint_angles[self.joint_names[2]] < 2:
            self.limb.set_joint_positions(joint_goal)

            # self.limb.set_joint_velocities(joint_vels)

            cur_joint_angles = self.limb.joint_angles()

            if cur_joint_angles[self.joint_names[2]] >= .75 and release == 0:
                    rospy.loginfo("Open Grippers")
                    self.OpenGripper()
                    release = 1
