#!/usr/bin/env python

""" This client node is the main node to track Sawyer's states during cornhole games.

"""
import rospy
import sys
import time

from geometry_msgs.msg import (
	Pose,
	PoseWithCovariance)

from std_srvs.srv import (
	Empty, 
	EmptyResponse, 
	SetBool, 
	SetBoolResponse)

from the_mighty_sawyer import (
	find_true,
	get_params_from_yaml)

from the_mighty_sawyer import (
	MoveArm)

from the_mighty_sawyer.srv import (
	GetPose)

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


####################################################
# 	SERVICES :: sawyer_movement_server
####################################################

def tms_initialization():
	"""
	Higher-level function that initializes The Mighty Sawyer
	"""
	print("Initializing...")
	start_up_client()
	target_board_client()
	go_to_home_pos_client()
	print("Initialization complete.")

def start_up_client():
	"""
	This is the client for the Service 'start_up' provided by sawyer_movement_server
	"""
	srv_name = 'start_up'
	rospy.wait_for_service(srv_name)
	try:
		_srv_start_up = rospy.ServiceProxy(srv_name, Empty)
		print("I am performing initialization tasks...")
		_srv_start_up()
		rospy.sleep(1)
		return _srv_start_up

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def go_to_home_pos_client():
	"""
	This is the client for the Service 'go_to_home_pos' provided by sawyer_movement_server
	"""
	# if srv_name is not 'go_to_home_pos':
	# 	pass
	srv_name = 'go_to_home_pos'
	rospy.wait_for_service(srv_name)
	try:
		_srv_go_to_home_pos = rospy.ServiceProxy(srv_name, Empty)
		print("I am moving to my throwing position...")
		_srv_go_to_home_pos()
		rospy.sleep(1)
		return _srv_go_to_home_pos
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def target_board_client():
	"""
	This is the client for the Service 'target_board' provided by sawyer_movement_server
	"""
	
	srv_name = 'target_board'
	rospy.wait_for_service(srv_name)
	try:
		_srv_target_board = rospy.ServiceProxy(srv_name, Empty)
		print("I am going to attempt to target the cornhole board...")
		_srv_target_board()
		rospy.sleep(1)
		return _srv_target_board
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def overhand_throw_client(srv_name):
	"""
	This is the client for the Service 'overhand_throw' provided by sawyer_movement_server
	"""
	if srv_name is not 'overhand_throw':
		pass

	rospy.wait_for_service(srv_name)
	try:
		_srv_overhand_throw = rospy.ServiceProxy(srv_name, Empty)
		print("I am going to attempt to throw the beanbag...")
		_srv_overhand_throw()
		rospy.sleep(1)
		return _srv_overhand_throw
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def make_adjustments_client(srv_name):
	"""
	This is the client for the Service 'make_adjustments' provided by sawyer_movement_server
	"""
	if srv_name is not 'make_adjustments':
		pass

	# _sub_pose = rospy.Subcriber('/pose_april_tags', PoseWithCovariance, pose_callback)

	# _pose_board = Pose()
	# _pose_bag = Pose()

	go_to_home_pos_client()

	rospy.wait_for_service(srv_name)
	try:
		_srv_make_adjustments = rospy.ServiceProxy(srv_name, Empty)
		print("I am going to attempt to throw the beanbag...")
		_srv_make_adjustments()
		rospy.sleep(1)
		return _srv_make_adjustments
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


	# def pose_callback():
	# 	pass


####################################################
# 	SERVICES :: grab_bag_server
####################################################

def grab_bag_client(srv_name):
	"""
	This is the client for the Service 'grab_bag' provided by grab_bag_server
	"""
	if srv_name is not 'grab_bag':
		pass

	rospy.wait_for_service(srv_name)
	try:
		_srv_grab_bag = rospy.ServiceProxy(srv_name, Empty)
		print("I am attempting to grab the beanbag...")
		_srv_grab_bag()
		rospy.sleep(1)
		return _srv_grab_bag
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


####################################################
# 	SERVICES :: score_keeping_server???
####################################################

# def evaluate_throw_result_client(srv_name):
# 	"""
# 	This is the client for the Service 'evaluate_throw_result' provided by 
# 	"""
# 	rospy.wait_for_service(srv_name)
# 	try:
# 		_srv_evaluate_throw_result = rospy.ServiceProxy(
# 										srv_name, 
# 										EvaluateThrowResult)
# 		print("I am going to evaluate my throw...")
# 		_srv_evaluate_throw_result()
# 		rospy.sleep(1)
# 		return _srv_evaluate_throw_result
# 	except rospy.ServiceException, e:
# 		print "Service call failed: %s"%e


def sawyer_main_client():
	"""
	This is the sawyer's main Service client that governs all of sawyer's Services
	"""

	#== initialization
	rospy.init_node("sawyer_main_client")

	#-- preserve this order until we use something like dictionaries
	_srv_names = [
						'grab_bag',
						'overhand_throw',
						# 'evaluate_throw_result',
						'make_adjustments']

	num_of_states = len(_srv_names)
	state_idx = 1			
	sawyer_state = _srv_names[state_idx]

	tms_initialization()
	
	#== main game loop
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		print("The current state is: " + str(sawyer_state))
		print("state_idx: " + str(state_idx))
		if (sawyer_state not in _srv_names):
			tms_initialization()
			
		elif (sawyer_state is 'grab_bag'):
			grab_bag_client(sawyer_state)
			# update_state()
			state_idx = (state_idx + 1) % num_of_states
			sawyer_state = _srv_names[state_idx]
			print("I am now ready to transition to state: " + str(sawyer_state))

		# elif (sawyer_state is 'target_board'):
		# 	target_board_client(sawyer_state)
		# 	# update_state()
		# 	state_idx = (state_idx + 1) % num_of_states
		# 	sawyer_state = _srv_names[state_idx]
		# 	print("I am now ready to transition to state: " + str(sawyer_state))
		
		# elif (sawyer_state is 'go_to_home_pos'):
		# 	go_to_home_pos_client(sawyer_state)
		# 	# update_state()
		# 	state_idx = (state_idx + 1) % num_of_states
		# 	sawyer_state = _srv_names[state_idx]
		# 	print("I am now ready to transition to state: " + str(sawyer_state))

		elif (sawyer_state is 'overhand_throw'):
			overhand_throw_client(sawyer_state)
			# update_state()
			state_idx = (state_idx + 1) % num_of_states
			sawyer_state = _srv_names[state_idx]
			print("I am now ready to transition to state: " + str(sawyer_state))

		# elif (sawyer_state is 'evaluate_throw_result'):
		# 	evaluate_throw_result_client(sawyer_state)
		# 	# state_idx, sawyer_state = update_state(state_idx, sawyer_state)
		# 	state_idx = (state_idx + 1) % num_of_states
		# 	sawyer_state = _srv_names[state_idx]
		# 	print("I am now ready to transition to state: " + str(sawyer_state))

		#-- make_adjustments
		elif (sawyer_state is 'make_adjustments'):
			make_adjustments_client(sawyer_state)
			# update_state()
			state_idx = (state_idx + 1) % num_of_states
			sawyer_state = _srv_names[state_idx]
			print("I am now ready to transition to state: " + str(sawyer_state))

		rate.sleep()

	# def update_state():
	# 	state_idx = (state_idx + 1) % num_of_states
	# 	sawyer_state = _srv_names[state_idx]
	# 	print("I am now ready to transition to state: " + str(sawyer_state))
				

if __name__ == '__main__':
	try:
		sawyer_main_client()
	except rospy.ROSInterruptException:
		pass