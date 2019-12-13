#!/usr/bin/env python
"""
bsed on: https://github.com/m-elwin/me495_practices
Test if throuw has hit the board
Sets team to red - use red bags
apriltags.launch and grab_bag.launch should have been run before running this test
Make sure board apriltag is visible
"""
import unittest
import rospy, math
from the_mighty_sawyer.srv import SetTeam, TagPose
import std_msgs.msg
from std_srvs.srv import Empty
from the_mighty_sawyer.tms_helper_functions import get_dist


class HardCaseNode(unittest.TestCase):
    def __init__(self, *args):
        super(HardCaseNode, self).__init__(*args)
        rospy.init_node("board_moved")
        self.thresh = .5

    def test_hit_board(self):
        """
        tests if last bag registers as hitting the board
        make sure board tag is visible when running this test
        place bag then run test
        """
        rospy.wait_for_service('locate_board')
        rospy.wait_for_service('set_team')

        rospy.ServiceProxy('set_team', SetTeam).call('red')
        loc_board = rospy.ServiceProxy('locate_board', TagPose)
        get_bag = rospy.ServiceProxy('locate_recent_bag', TagPose)
        clear_board = rospy.ServiceProxy('clear_board', Empty)
        clear_board()

        board_pos = loc_board()
        rospy.loginfo(board_pos)
        rospy.loginfo('Press ENTER when bag in position (in terminal where apriltags was run')
        bag_pos = get_bag()
        rospy.loginfo(bag_pos)

        if not bag_pos.pose.position.x and not bag_pos.pose.position.y and not bag_pos.pose.position.z:
            rospy.loginfo('no bag detected. assume in hole')
            self.assertTrue(True)
        else:
            dist = get_dist(board_pos.pose, bag_pos.pose)
            rospy.loginfo(dist)
            self.assertLess(dist, self.thresh)


if __name__ == "__main__":
    unittest.main()
