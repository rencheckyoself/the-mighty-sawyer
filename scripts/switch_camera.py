#!/usr/bin/env python

import sys
import imagelib

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: camera_switcher active_camera (head, arm)")
    else:
        imagelib.switch_camera(sys.argv[1])