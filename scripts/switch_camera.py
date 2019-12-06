#!/usr/bin/env python

import sys
from the_mighty_sawyer.imagelib import switch_camera
import rospy

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: camera_switcher active_camera (head, arm)")
    else:
        rospy.init_node('switch_camera')
        switch_camera(sys.argv[1])