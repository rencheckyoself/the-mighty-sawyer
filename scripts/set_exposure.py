#!/usr/bin/env python

"""
Script to display a png from images/display folder on the screen
"""

import sys
from the_mighty_sawyer.imagelib import set_exposure
import rospy

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("args: cam val\ncam: arm head")
    else:
        rospy.init_node('set_exposure')
        set_exposure(sys.argv[1], sys.argv[2])