#!/usr/bin/env python
"""
bsed on: https://github.com/m-elwin/me495_practices
allows testing of board locations
"""
import unittest
import rospy, math
from the_mighty_sawyer.srv import SetTeam, TagPose
import std_msgs.msg
from std_srvs.srv import Empty, EmptyResponse


def get_dist(pose1, pose2):
    """
    Calculates the Euclidean distance between two poses.
    """
    return math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)


class HardCaseNode(unittest.TestCase):
    def __init__(self, *args):
        super(HardCaseNode, self).__init__(*args)
        rospy.init_node("board_moved")
        self.thresh = .4

    def test_board_moved(self):
        """
        tests if the board has moved
        place board, run test, press enter to test if new pos within move thresh
        """
        loc_board = rospy.ServiceProxy('locate_board', TagPose)
        pos = loc_board.call(Empty)
        rospy.loginfo(pos)
        raw_input('Press enter when board in second position.')
        new_pos = loc_board.call(Empty)
        rospy.loginfo(new_pos)
        dist = get_dist(pos, new_pos)
        rospy.loginfo(dist)
        self.assertLess(dist, self.thresh)


if __name__ == "__main__":
    import rostest
    rostest.rosrun('me495_practices', "hard_case_node", HardCaseNode)
