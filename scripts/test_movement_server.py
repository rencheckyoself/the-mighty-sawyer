#!/usr/bin/env python

"""
This file acts as a unit test for the sawyer movement server services. This
should perform a generic overhand throw.
"""

from __future__ import division
import numpy as np
import rospy

from the_mighty_sawyer import MoveArm

import intera_interface
from intera_interface import CHECK_VERSION

import geometry_msgs.msg
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool


def main():

    # Launch Node
    rospy.init_node('traj_with_sawyer', anonymous=True)

    # Initialize Services
    rospy.wait_for_service('start_up')
    step1 = rospy.ServiceProxy('start_up', Empty)

    rospy.wait_for_service('go_to_home_pos')
    step2 = rospy.ServiceProxy('go_to_home_pos', Empty)

    rospy.wait_for_service('overhand_throw')
    step3 = rospy.ServiceProxy('overhand_throw', Empty)

    rospy.wait_for_service('actuate_gripper')
    gripper = rospy.ServiceProxy('actuate_gripper', SetBool)

    # Set Up Robot
    step1()

    rospy.sleep(1)

    # Open Gripper
    gripper(1)

    rospy.sleep(1)

    # Go to home position
    step2()

    rospy.sleep(1)

    # Close Gripper
    go = gripper(0)

    rospy.sleep(1)

    # Throw
    go = step3()

    rospy.sleep(1)

    # Go to home
    go = step2()

    rospy.sleep(1)

    print("Done")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
