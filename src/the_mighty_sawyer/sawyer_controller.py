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
        joint_goal = {'right_j0':0.003546875,
                      'right_j1':-0.5124091796875,
                      'right_j2':0.0377041015625,
                      'right_j3':1.8277978515625,
                      'right_j4':-0.048537109375,
                      'right_j5':-2.8403095703125,
                      'right_j6':-1.39}

        self.limb.set_joint_position_speed(speed=0.3)

        rospy.loginfo("Going to Home...")

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

    def do_under_hand_toss(self):

        self.underhand_throw_start()
        rospy.sleep(1)
        self.underhand_throw()

    def underhand_throw_start(self):
        """
        Moves the arm to a calculated position to begin throwing the bag.
        """

        rospy.loginfo("Start Underhand Throw")

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


        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.move_to_joint_positions(joint_goal)

        rospy.sleep(.5)

        joint_goal = {'right_j2':-1.5}

        # joint_goal = {'right_j2':-.5}

        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.move_to_joint_positions(joint_goal)

    def underhand_throw(self):
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

    def overhand_throw_start(self):
        """
        Get to over hand throw position
        """
        rospy.loginfo("Start Overhand Throw")

        cur_joint_angles = self.limb.joint_angles()

        joint_goal = {'right_j0':-3,
                      'right_j1':0.01,
                      'right_j2':-3.04,
                      'right_j3':-1.75,
                      'right_j4':-0.074908203125,
                      'right_j5':-2}

        self.limb.set_joint_position_speed(speed=0.2)
        self.limb.move_to_joint_positions(joint_goal, timeout=30)

    def overhand_throw(self, option=1):


        if option == 1:

            joint_goal = {'right_j1':-3,
                          'right_j3':1.5,
                          'right_j5':1}
        else:
            joint_goal = {'right_j1':-.1,
                          'right_j3':.1,
                          'right_j5':.1}


        self.limb.set_joint_position_speed(speed=1)

        release = 0
        cur_joint_angles = self.limb.joint_angles()

        while cur_joint_angles[self.joint_names[1]] > -2.8:

            if option == 1:
                self.limb.set_joint_positions(joint_goal)
            else:
                self.limb.set_joint_velocities(joint_goal)

            cur_joint_angles = self.limb.joint_angles()

            if cur_joint_angles[self.joint_names[1]] <= -1.3 and release == 0:
                    rospy.loginfo("Open Grippers")
                    self.OpenGripper()
                    release = 1

                    if option != 1:
                        joint_goal = {'right_j1':0,
                                      'right_j3':0,
                                      'right_j5':0}
