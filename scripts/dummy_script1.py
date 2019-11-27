#!/usr/bin/env python

"""
Scripts in this folder act mainly as a lightweight script that imports stuff from the subpackage (eg. the-mighty-sawyer/src/the_mighty_sawyer) -- where all the cool stuff is really happening.
"""
from __future__ import print_function
from the_mighty_sawyer import function_x_from_dummy_module1, Class_Y_from_dummy_module1

def lightweight():
	pass

if __name__ == "__main__":
	lightweight()
	function_x_from_dummy_module1()		# this is a function from dummy_module1