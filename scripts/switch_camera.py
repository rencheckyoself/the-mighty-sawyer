#!/usr/bin/env python

import sys
import imagelib
import rospy

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: camera_switcher active_camera (head, arm)")
    else:
        rospy.init_node('switch_camera')
        imagelib.switch_camera(sys.argv[1])