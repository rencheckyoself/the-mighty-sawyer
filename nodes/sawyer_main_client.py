#!/usr/bin/env python

""" This node is used to track Sawyer's states during cornhole games.
	
	SERVICES:
		'wait_for_bag' [0]
		'grab_bag' [1]
		'move_to_throw_position [2]
		'execute_throw' [3]
		'evaluate_throw_result' [4]


"""
import rospy
import sys

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
		print("I am waiting for my beanbag...")
		return _srv_start_up()

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def actuate_gripper_client(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_actuate_gripper = rospy.ServiceProxy(
										srv_name, 
										Empty)
		print("I am grabbing the beanbag...")
		return _srv_actuate_gripper()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def go_to_home_pos_client(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_go_to_home_pos = rospy.ServiceProxy(
										srv_name, 
										Empty)
		print("I am moving to my throwing position...")
		return _srv_go_to_home_pos()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def overhand_throw(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_overhand_throw = rospy.ServiceProxy(
										srv_name, 
										SetBool)
		print("I am going to attempt to throw the beanbag...")
		return _srv_go_to_home_pos()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def evaluate_throw_result(srv_name):
	rospy.wait_for_service(srv_name)
	try:
		_srv_evaluate_throw_result = rospy.ServiceProxy(
										srv_name, 
										EvaluateThrowResult)
		print("I am going to evaluate my throw...")
		return _srv_evaluate_throw_result()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def sawyer_main_client():

	#== initialization

	#==	TODO: fix this bug --> use param server
	# this apparently is relative to where you rosrun the node, 
	# not relative to the project path
	_fn = 'config//joints_cfg_sawyer.yaml'
	_params = get_params_from_yaml(_fn)
	_receive_home = _params.get('receive_home')
	_throw_home = _params.get('throw_home')

	_srv_names = [
						'start_up',
						'actuate_gripper',
						'go_to_home_pos',
						'overhand_throw',
						'evaluate_throw_result']

	sawyer_state = _srv_names[0]

	rospy.loginfo("Initialization complete.")
	rospy.loginfo("The current state is: %s.", sawyer_state)


	while not rospy.is_shutdown():
		
		if (sawyer_state is 'start_up'):
			start_up_client(sawyer_state)
			sawyer_state = _srv_names[1]
		elif (sawyer_state is 'actuate_gripper'):
			actuate_gripper_client(sawyer_state)
			sawyer_state = _srv_names[2]
		elif (sawyer_state is 'go_to_home_pos'):
			actuate_gripper_client(sawyer_state)
			sawyer_state = _srv_names[3]
		elif (sawyer_state is 'overhand_throw'):
			actuate_gripper_client(sawyer_state)
			sawyer_state = _srv_names[4]
		elif (sawyer_state is 'evaluate_throw_result'):
			actuate_gripper_client(sawyer_state)
			sawyer_state = _srv_names[0]

		rospy.sleep(0.01)

if __name__ == '__main__':
	try:
		sawyer_main_client()
	except rospy.ROSInterruptException:
		pass