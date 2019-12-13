#!/usr/bin/env python
"""
bsed on: https://github.com/m-elwin/me495_practices
allows testing if last throw registers as hitting the board
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
        self.thresh = .5

    def test_hit_board(self):
        """
        tests if last bag registers as hitting the board
        make sure board tag is visible when running this test
        place bag then run test
        """
        rospy.ServiceProxy('set_team', SetTeam).call(std_msgs.msg.String('red'))
        loc_board = rospy.ServiceProxy('locate_board', TagPose)
        get_bag = rospy.ServiceProxy('locate_recent_bag', TagPose)

        board_pos = loc_board.call(Empty)
        rospy.loginfo(board_pos)
        bag_pos = get_bag.call(Empty)
        rospy.loginfo(bag_pos)

        if not bag_pos.position.x and not bag_pos.position.y and not bag_pos.position.z:
            rospy.loginfo('no bag detected. assume in hole')
            self.assertTrue(True)
        else:
            dist = get_dist(board_pos, bag_pos)
            rospy.loginfo(dist)
            self.assertLess(dist, self.thresh)


if __name__ == "__main__":
    import rostest
    rostest.rosrun('me495_practices', "hard_case_node", HardCaseNode)
