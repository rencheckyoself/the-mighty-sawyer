#!/usr/bin/env python
"""
bsed on: https://github.com/m-elwin/me495_practices
tests if board has remained stationary
apriltags.launch should have been run before running this test
"""
import unittest
import rospy, math
from the_mighty_sawyer.srv import SetTeam, TagPose
from the_mighty_sawyer.tms_helper_functions import get_dist


class HardCaseNode(unittest.TestCase):
    def __init__(self, *args):
        super(HardCaseNode, self).__init__(*args)
        rospy.init_node("board_moved")
        self.thresh = .4

    def test_board_moved(self):
        """
        tests if the board has remained stationary
        place board, run test, press enter to test if new pos within move thresh
        """
        rospy.wait_for_service('locate_board')
        loc_board = rospy.ServiceProxy('locate_board', TagPose)
        pos = loc_board()
        rospy.loginfo(pos)
        raw_input('Press enter when board in second position.')
        new_pos = loc_board()
        rospy.loginfo(new_pos)
        dist = get_dist(pos.pose, new_pos.pose)
        rospy.loginfo(dist)
        self.assertLess(dist, self.thresh)


if __name__ == "__main__":
    unittest.main()
