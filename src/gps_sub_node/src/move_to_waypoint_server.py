#! /usr/bin/env python

# import necessary pacakges
import rospy
import actionlib
from gps_node import Waypoints, GPSNode
from actionlib.msg import TestFeedback, TestResult, TestAction
from geometry_msgs.msg import Twist

"""
	Test.action
	int32 goal # next way point from client
	---
	int32 result # final value when we are less than distance margin
	---
	int32 feedback # returns distance to current
"""

class MovingWaypoints:
	def __init__(self, serverTopic="/move_waypoint_server"):
		# initialize 
		rospy.init_node("move_waypoints_server_node")

		# initialize feedback, result
		self._feedback = TestFeedback()
		self._result = TestResult()

		# Action server initialization
		self._as = actionlib.SimpleActionServer(serverTopic, TestAction, self.goal_callback, False)
		self._as.start()

		# log information
		rospy.loginfo("waypoint server initialized")

		# initialize gps class -> has current and dest node. current_wp and destination_wp
		self.gps_class = GPSNode() 

		# set error margin and ros rate
		self._action_rate = 10
		self.distance_margin = 2 #[meters]

		# initialize cmd publisher
		self.cmd_pub = rospy.Publisher("/catvehicle/cmd_vel_safe", Twist, queue_size=1)


	# recieve goal from client and calculate distance
	def goal_callback(self, goal):
		# set ros rate and initialize success
		r = rospy.Rate(self._action_rate)
		success = True

		# set destination goal
		self.gps_class.set_destination(goal.goal)

		# check is the destination is not null
		if self.gps_class.destination_wp != None:

			# calculate distance between the way points
			distance_ = self.gps_class.waypoint_dist(unit="m")

			# set feed back and publish the feedback
			self._feedback = int(distance_)
			self._as.publish_feedback(self._feedback)

			# publish information for user
			rospy.loginfo("Start from waypoint origin:{} to destination waypoint: {}".format(self.gps_class.current_wp,
			 self.gps_class.destination_wp.print()))

			# need twist to move vehicle forward
			cmd_msg = Twist()
			cmd_msg.linear.x = 1.0

			# start moving to the waypoint
			while int(distance_) >= int(self.distance_margin):
				# print distance 
				rospy.loginfo("Distance from destination: {} [m]".format(distance_))

				# check that server cancellation not requested by client
				if self._as.is_preempt_requested():
					rospy.loginfo("Waypoint has been cancelled")

					# set in preempted state
					self._as.set_preempted()
					success = False
					self._result.result = int(success)

				# set feed back and publish the feedback
				self._feedback = int(distance_)
				self._as.publish_feedback(self._feedback)

				# publisj the forward move command
				self.cmd_pub.publish(cmd_msg)
				rospy.logwarn("Moving Car")

				# put too sleep
				r.sleep()

				# update the distance
				distance_ = self.gps_class.waypoint_dist(unit="m")


			# stop car moobing
			rospy.logwarn("Car stopping")
			cmd_msg.linear.x = 0.0
			self.cmd_pub.publish(cmd_msg)

		else:
			success = False

		# either goal has been achieved (success == True) -> print final result
		# or client preempterd the goal (success == false)
		if success:
			rospy.loginfo("successfully moved from waypoint: {} to waypoint: {}".format(self.gps_class.current_wp, self.gps_class.destination_wp))
			rospy.loginfor("Distance from waypoint: {} [m]".format(int(distance_)))

		else:
			rospy.loginfo("Waypoint movement Failed")

		# set result
		self._result.result = int(success)
		self._as.set_succeeded(self._result)


if __name__ == "__main__":
	MovingWaypoints()
	rospy.spin()