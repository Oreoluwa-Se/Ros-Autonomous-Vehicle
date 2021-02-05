#! /usr/bin/env python

# import necessary packages
import rospy
import time
import actionlib
from actionlib.msg import TestGoal, TestAction, TestFeedback, TestResult

"""
	Test.action
	int32 goal # next way point from client
	---
	int32 result # final value when we are less than distance margin
	---
	int32 feedback # returns distance to current
"""

# create class for client
class WaypointClient:
	def __init__(self, serverTopic="/move_waypoint_server"):
		# intialize node
		rospy.init_node("move_waypoint_client_node")

		# constants for action class
		self.state_dict = {0: "pending", 1: "active", 2: "done", 3: "warn", 4: "error"}

		# start server send goal
		self.server_start(serverTopic)

		# intialize the rospy rate
		self._action_rate = 10

		
	# starts server and sends goal
	def server_start(self, serverTopic):
		# create server .. and wait
		self.client = actionlib.SimpleActionClient(serverTopic, TestAction)
		rospy.loginfo("Waiting for service: {}".format(serverTopic))
		self.client.wait_for_server()
		rospy.loginfo("{} service found".format(serverTopic))

		# create goal
		goal = TestGoal()
		goal.goal = 1 # indicates which way point to move to

		# send goal
		self.client.send_goal(goal, feedback_cb=self.feedback)


	def feedback(self, feedback):
		rospy.loginfo("Distance to Waypoint: {}".format(feedback.feedback))

		# initialize the rate
		rate = rospy.Rate(self._action_rate)

		# extract the client state -> can also use simple_state -> returns 1 when active and 2 when finished
		state_result = self.client.get_state()

		# print current state
		rospy.loginfo("Server current state: {}".format(self.state_dict[state_result]))

		val_list = list(self.state_dict.values())

		# while state is pending or active
		while state_result < val_list("done"):
			# can do other things here
			rate.sleep()
			state_result = self.client.get_state()

		rospy.loginfo("Final state: {}".format(self.state_dict[state_result]))

		if state_dict[state_result] == state_dict[4]:
	    	rospy.logerr("Something went wrong in the Server Side")
		if state_dict[state_result] == state_dict[3]:
		    rospy.logwarn("There is a warning in the Server Side")




