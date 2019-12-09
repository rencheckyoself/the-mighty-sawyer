#!/usr/bin/env python

"""
Script to display a png from images/display folder on the screen
"""

import sys
from the_mighty_sawyer.imagelib import switch_camera, display_png
import rospy

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("args: img b g r")
    else:
        rospy.init_node('display_png')
        display_png(sys.argv[1],(sys.argv[2], sys.argv[3], sys.argv[4]))