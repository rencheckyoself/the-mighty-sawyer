#!/usr/bin/env python
from __future__ import print_function
import yaml
import sys
import os
import math

# def Class_Y_from_dummy_module1():
# 	def d(self):
# 		""" Dummy class """
# 		return "This is a dummy Class_Y_from_dummy_module1 class. This demonstrates how one can import a class from a python subpackage"

# def function_x_from_dummy_module1():
# 	""" Dummy function """
# 	print("This demonstates how one can import a function from a python subpackage")

def get_params_from_yaml(filename):
	"""
	Opens yaml file (from the relative path) and returns the data
	"""
	with open(filename) as f:
	  # use safe_load instead load
	  data = yaml.safe_load(f)
	return data

def find_true(alist):
	"""
	Find the indices in a list where they are Trues
	"""
	return [i for i, x in enumerate(alist) if x]
	# return [alist.index(i) for i in alist if i == True]	#-- just grabs the first True

def calculate_l2_norm(xy0, xy1):
	pass

def calculate_bearing():
	pass