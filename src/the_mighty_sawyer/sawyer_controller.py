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

from geometry_msgs.msg import Pose
from std_msgs.msg import String

class MoveArm(object):
    """
    Class containing methods to fully interact with the sawyer arm.
    """
    def __init__(self):
        self.rs = intera_interface.RobotEnable(CHECK_VERSION)

        self.gripper = intera_interface.Gripper('right_gripper')

        self.limb = intera_interface.Limb()

        self.head = intera_interface.Head()
        self.head_angle = .5

        self.joint_names = self.limb.joint_names()

        self.target_pose = Pose()

        self.select_throw = 1

        self.underhand_throw_speed = 0.8
        self.underhand_target_angle = -1.5
        self.underhand_release_angle = .75

        self.overhand_throw_speed = 1
        self.overhand_target_angle = -3
        self.overhand_release_angle = -1.3

        self.overhand_throw_offset = 0

    def initializaton(self):
        """
        Higher-level function that performs Sawyer-related initializations.
        """
        self.EnableRobot()
        rospy.sleep(.5)
        self.InitializeGripper()

    def EnableRobot(self):
        """
        Enables the Sawyer robot.
        """
        self.rs.enable()

    def reset_default_settings(self):
        """
        This method is used to reset the class parameters to their initial
        values.
        """
        self.underhand_throw_speed = 0.8
        self.underhand_target_angle = -1.5
        self.underhand_release_angle = .75

        self.overhand_throw_speed = 1
        self.overhand_target_angle = -3
        self.overhand_release_angle = -1.3

    def InitializeGripper(self):
        """
        Calls the sequence of methods to calibrate the gripper. This should be
        at the beginning of every launch.
        """
        rospy.loginfo("Initializing Gripper...")

        self.gripper.reboot()
        rospy.sleep(1)
        self.gripper.calibrate()

    def actuate_gripper(self, state):
        """
        Method to actuate the grippers given an input.

        INPUT:
            state (bool): value of 1 to open the grippers and 0 to close
        """

        if state.data == 1:
            self.OpenGripper()
        elif state.data == 0:
            self.CloseGripper()

    def OpenGripper(self):
        """
        Calls the method to open the gripper.
        """
        rospy.loginfo("Gripper Open")
        self.gripper.open()

    def CloseGripper(self):
        """
        Calls the method to close the gripper.
        """
        rospy.loginfo("Gripper Close")
        self.gripper.close()

    def go_to_home_pos(self):
        """
        Moves the arm to a preconfigured home position by setting joint
        angles directly.

        This position has the arm folded up infront of the base with the arm
        camera facing forward. The arm camera will be used to start searching
        for a bean bag.
        """

        # Joint Angles for home position
        joint_goal = {'right_j0':-0.25973828125,
                      'right_j1':0.727921875,
                      'right_j2':1.42190625,
                      'right_j3':-1.5519775390625,
                      'right_j4':-0.749162109375,
                      'right_j5':-1.46358984375,
                      'right_j6':-1.437654296875}

        # # OG Home Position
        # joint_goal = {'right_j0':0.003546875,
        #               'right_j1':-0.5124091796875,
        #               'right_j2':0.0377041015625,
        #               'right_j3':1.8277978515625,
        #               'right_j4':-0.048537109375,
        #               'right_j5':-2.8403095703125,
        #               'right_j6':-1.39}

        # Set motion speed
        self.limb.set_joint_position_speed(speed=0.3)

        rospy.loginfo("Going to Home...")

        # Send command
        self.limb.move_to_joint_positions(joint_goal)

        self.head.set_pan(self.head_angle)

    def go_to_bag_pos(self, delay_time):
        """
        DEPRICIATED
        Move the wrist area forward by 90deg. from the home position. Ideally,
        this is called when the robot is already located at the home position.

        This method is called to indicate that sawyer has identified a bean bag
        using the arm camera and will wait for 3 second for a user to position
        a bag between its gripper.

        INPUT:
        delay_time: amount of seconds sawyer will give the user to position a
        bag. Should be any number > 0
        """

        joint_goal = {'right_j0':0.003546875,
                      'right_j1':-0.5124091796875,
                      'right_j2':0.0377041015625,
                      'right_j3':1.8277978515625,
                      'right_j4':-0.048537109375,
                      'right_j5':-1.5,
                      'right_j6':-1.39}

        self.limb.set_joint_position_speed(speed=0.1)
        self.limb.move_to_joint_positions(joint_goal)

        rospy.loginfo("Waiting for bag...")

        rospy.sleep(delay_time)

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

    def do_under_hand_toss(self, release_angle=None, throwing_speed=None, target_angle=None):
        """
        Calls the the subsequent functions to throw the bag with an underhanded
        throw. This is good for up to a 3 inch throw outside of sawyers range.

        INPUT:
            release_angle: Change the angle threshold that the gripper will open
            at during the throwing motion

            throwing_speed: Set the ratio for the speed of all joints during the
            throwing motion

            target_angle: Change the angle of the base joint to point the
            throwing motion in a different direction
        """

        if throwing_speed is not None:

            if throwing_speed > 1:
                rospy.logerr("Throwing speed must be between 0 and 1. Value set \
                              to closest bound")
                throwing_speed = 1
            elif throwing_speed < 0:
                rospy.logerr("Throwing speed must be between 0 and 1. Value set \
                              to closest bound")
                throwing_speed = 0

            self.underhand_throw_speed = throwing_speed

        if release_angle is not None:
            self.underhand_release_angle = release_angle

        if target_angle is not None:
            self.underhand_target_angle = target_angle

        self.underhand_throw_start()
        rospy.sleep(1)
        self.underhand_throw()

    def underhand_throw_start(self):
        """
        Moves the arm to a preset position to prepare for an underhanded throw.

        This motion is done in two stages to avoid motions close to the body.
        """

        rospy.loginfo("Start Underhand Throw")

        # Get Current Joint Angle Values
        cur_joint_angles = self.limb.joint_angles()

        # Anlge position values
        joint_goal = {'right_j0':self.underhand_target_angle,
                      'right_j1':0.0190947265625,
                      'right_j2':cur_joint_angles[self.joint_names[2]],
                      'right_j3':1.5639794921875,
                      'right_j4':-1.60975390625,
                      'right_j5':2.5}

        # Position that is not fully drawn back
        # joint_goal = {'right_j0':-1.5,
        #               'right_j1':0.0190947265625,
        #               'right_j2':cur_joint_angles[self.joint_names[2]],
        #               'right_j3':1.5639794921875,
        #               'right_j4':-1.60975390625,
        #               'right_j5':1}

        # Actuate arm
        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.move_to_joint_positions(joint_goal)

        rospy.sleep(.5)

        # Second motion joint position values
        joint_goal = {'right_j2':-1.5}

        # Position that is not fully drawn back
        # joint_goal = {'right_j2':-.5}

        # Actuate Arm
        self.limb.set_joint_position_speed(speed=0.3)
        self.limb.move_to_joint_positions(joint_goal)

    def underhand_throw(self):
        """
        This function actuates the arm to throw underhanded. It should be called
        directly after the underhand throw start position.
        """

        rospy.loginfo("Throwing...")

        # Get Current Joint Angles
        cur_joint_angles = self.limb.joint_angles()

        # Final Position to End Throw
        joint_goal = {'right_j2':2,
                      'right_j5':-.75}

        # Set throwing speed
        self.limb.set_joint_position_speed(speed=self.underhand_throw_speed)

        release = 0

        # While the arm has not reach the end of it's movement...
        while cur_joint_angles[self.joint_names[2]] < 2:

            # Actuate Joint
            self.limb.set_joint_positions(joint_goal)

            cur_joint_angles = self.limb.joint_angles()

            # Check if joint has passed the release threshold
            if cur_joint_angles[self.joint_names[2]] >= self.underhand_release_angle and release == 0:
                    self.OpenGripper()
                    release = 1

    def do_over_hand_toss(self, release_angle=None, throwing_speed=None, target_angle=None):
        """
        Calls the the subsequent functions to throw the bag with an overhanded
        throw. This is good for up to a 3 ft throw outside of sawyers range.

        INPUT:
            release_angle: Change the angle threshold that the gripper will open
            at during the throwing motion

            throwing_speed: Set the ratio for the speed of all joints during the
            throwing motion

            target_angle: Change the angle of the base joint to point the
            throwing motion in a different direction
        """

        if throwing_speed is not None:

            if throwing_speed > 1:
                rospy.logerr("Throwing speed must be between 0 and 1. Value set \
                              to closest bound")
                throwing_speed = 1
            elif throwing_speed < 0:
                rospy.logerr("Throwing speed must be between 0 and 1. Value set \
                              to closest bound")
                throwing_speed = 0

            self.overhand_throw_speed = throwing_speed

        if release_angle is not None:
            self.overhand_release_angle = release_angle

        if target_angle is not None:
            self.overhand_target_angle = target_angle

        self.overhand_throw_start()
        rospy.sleep(1)
        self.overhand_throw()

    def overhand_throw_start(self):
        """
        Moves the arm to a preset position to prepare for an overhanded throw.

        This function only is able to target the area to sawyer left.
        """
        rospy.loginfo("Start Overhand Throw")

        # Get current Joint Angles
        cur_joint_angles = self.limb.joint_angles()

        calc_offset_j3 = -1.75 + self.overhand_throw_offset
        calc_offset_j5 = -2 + self.overhand_throw_offset

        # Set Desired Joint Anlges
        joint_goal = {'right_j0':self.overhand_target_angle,
                      'right_j1':0.01,
                      'right_j2':-3.04,
                      'right_j3':calc_offset_j3,
                      'right_j4':-0.074908203125,
                      'right_j5':calc_offset_j5}

        # Actuate Arm
        self.limb.set_joint_position_speed(speed=0.2)
        self.limb.move_to_joint_positions(joint_goal, timeout=30)

    def overhand_throw(self):
        """
        This function actuates the arm to throw overhanded. It should be called
        directly after the overhand throw start position.
        """

        # Set desired ending position
        joint_goal = {'right_j1':-3,
                      'right_j3':1.5,
                      'right_j5':1.25}

        # Set throwing speed
        self.limb.set_joint_position_speed(speed=self.overhand_throw_speed)

        # Get Current Joint Angles
        cur_joint_angles = self.limb.joint_angles()

        release = 0

        # While the arm has not reach the end of it's movement...
        while cur_joint_angles[self.joint_names[1]] > -2.8:

            # Actuate Joints
            self.limb.set_joint_positions(joint_goal)

            # Get Joint Angles
            cur_joint_angles = self.limb.joint_angles()

            # Check if joint has passed the release threshold
            if cur_joint_angles[self.joint_names[1]] <= -1.3 and release == 0:
                    rospy.loginfo("Open Grippers")
                    self.OpenGripper()
                    release = 1
