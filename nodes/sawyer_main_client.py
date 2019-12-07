#!/usr/bin/env python

""" This client node is the main node to track Sawyer's states during cornhole games.

"""
import rospy
import sys
import time

from the_mighty_sawyer import (
	find_true,
	get_params_from_yaml)

from the_mighty_sawyer import (
	MoveArm)

from std_srvs.srv import (
	Empty, 
	EmptyResponse, 
	SetBool, 
	SetBoolResponse)

# from the_mighty_sawyer.srv import (
# 	WaitForBag, 
# 	GrabBag2, 
# 	MoveToThrowPos,
# 	ExecuteThrow,
# 	EvaluateThrowResult,
# 	WaitForBagResponse, 
# 	GrabBag2Response, 
# 	MoveToThrowPosResponse,
# 	ExecuteThrowResponse,
# 	EvaluateThrowResultResponse)

def start_up_client(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_start_up = rospy.ServiceProxy(
											srv_name, 
											Empty)
		_srv_start_up()
		print("I am waiting for my beanbag...")
		rospy.sleep(1)
		return _srv_start_up

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def actuate_gripper_client(srv_name):
	_open_gripper = True
	rospy.wait_for_service(srv_name)
	try:
		_srv_actuate_gripper = rospy.ServiceProxy(
										srv_name, 
										SetBool)
		print("I am grabbing the beanbag...")
		_srv_actuate_gripper(_open_gripper)
		rospy.sleep(1)
		_srv_actuate_gripper(not _open_gripper)
		return _srv_actuate_gripper
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def go_to_home_pos_client(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_go_to_home_pos = rospy.ServiceProxy(
										srv_name, 
										Empty)
		print("I am moving to my throwing position...")
		_srv_go_to_home_pos()
		return _srv_go_to_home_pos
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def overhand_throw_client(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_overhand_throw = rospy.ServiceProxy(
										srv_name, 
										Empty)
		print("I am going to attempt to throw the beanbag...")
		_srv_overhand_throw()
		return _srv_overhand_throw
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# def evaluate_throw_result_client(srv_name):
# 	rospy.wait_for_service(srv_name)
# 	try:
# 		_srv_evaluate_throw_result = rospy.ServiceProxy(
# 										srv_name, 
# 										EvaluateThrowResult)
# 		print("I am going to evaluate my throw...")
# 		return _srv_evaluate_throw_result
# 	except rospy.ServiceException, e:
# 		print "Service call failed: %s"%e


def sawyer_main_client():

	#== initialization

	#==	TODO: fix this bug --> use param server
	# this apparently is relative to where you rosrun the node, 
	# not relative to the project path
	# _fn = 'config//joints_cfg_sawyer.yaml'
	# _params = get_params_from_yaml(_fn)
	# _receive_home = _params.get('receive_home')
	# _throw_home = _params.get('throw_home')

	rospy.init_node("sawyer_main_client")
	_srv_names = [
						'start_up',
						'actuate_gripper',
						'go_to_home_pos',
						'overhand_throw',
						'evaluate_throw_result']

	sawyer_state = _srv_names[0]

	print("Initialization complete.")
	
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		print("The current state is: " + str(sawyer_state))
		if (sawyer_state not in _srv_names):
			sawyer_state = _srv_names[0]
			rospy.sleep(1)

		elif (sawyer_state is 'start_up'):
			start_up_client(sawyer_state)
			sawyer_state = _srv_names[1]
			
		elif (sawyer_state is 'actuate_gripper'):
			actuate_gripper_client(sawyer_state)
			sawyer_state = _srv_names[2]
			print("I am now ready to transition to state: " + str(sawyer_state))
		
		elif (sawyer_state is 'go_to_home_pos'):
			go_to_home_pos_client(sawyer_state)
			sawyer_state = _srv_names[3]
			print("I am now ready to transition to state: " + str(sawyer_state))

		elif (sawyer_state is 'overhand_throw'):
			overhand_throw_client(sawyer_state)
			sawyer_state = _srv_names[0]
			print("I am now ready to transition to state: " + str(sawyer_state))

		# elif (sawyer_state is 'evaluate_throw_result'):
		# 	evaluate_throw_result_client(sawyer_state)
		# 	sawyer_state = _srv_names[0]
		# 	print("I am now in state: " + str(sawyer_state))
		# 	rospy.sleep(1)

		rate.sleep()

if __name__ == '__main__':
	try:
		sawyer_main_client()
	except rospy.ROSInterruptException:
		pass