#! /usr/bin/env python

# import necessary packages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import tf.transformations import euler_from_quaternion, quartenion_from_euler
from helpers.gps_node_init import GpsNode
import actionlib
import rospy
import time

class MoveBaseClient:
  def __init__(self):
    # initialize node
    rospy.init_node("move_waypoint_node")

    # constants for the action class
    self.state_ = {0: "pending", 1: "active", 2: "done", 3: "warn", 4: "error"}

    # initialize gps node
    self.gps_node = GpsNode()

    # connect to the server
    self.server_conect("/move_base")

  # connect to movebase server target[[lat, lon]]
  def server_connect(self, topic, target=[[39.50920331, -0.4659816],[39.5080331, -0.4619816]]):
          
    # create server, wait, print message
    self.client = actionlib.SimpleActionClient(topic, MoveBaseAction)
    rospy.loginfo("Waiting for service: {}".format(topic))
    self.client.wait_for_server()
    rospy.loginfo("{} service found".format(topic))

    # create goal
    goal = MoveBaseGoal()

    for i in range(len(target)):
      # create destination waypoint
      self.gps_node.dest_init(target[i][0], target[i][1], alt=0)

      # get x, y, delta_x, delta_y
      x, y, delta_x,  delta_y = self.gps_node.get_xy_from_lat_long()

      # stat sheet -> target pose location to general odometry frame
      goal.target_pose.frame_id = "odom"
      goal.target_pose.stamp    = rospy.get_time()

      # x and y locations
      goal.target_pose.pose.position.x = x
      goal.target_pose.pose.position.y = y
      
      # before getting to final position we calculate the rotation between points
      if target[i] != target[-1]:
      else:


      goal.target_pose.pose.orientation.z = 0
      goal.target_pose.pose.orientation.w = 1.0

      # send goal and initiate feedback loop
      self.client.send_goal(goal, feedback_cb=self.feedback)

  def point_manip(self, point):
    # 



        
  # feedback and tells distance to current goal
  def feedback(self, feedback):
    # rospy rate
    rate = rospy.Rate(10)
       
    # distance to location
    self.gps_node.waypoint_distance()

    # state result
    state_res = self.client.get_state()

    # get current state
    rospy.loginfo("Server current state: {}".format(self.state_[state_res]))

    val_list = list(self.state_.values())

    # while state is pending or active
    while state_res < val_list.index("done"):
      # print current distance to destination
      self.gps_node.waypoint_distance()

      # sleep and extract result
      rate.sleep()
      state_res = self.client.get_state()

    rospy.loginfo("Final state: {}".format(self.state_[state_res]))

    if state_dict[state_res] == state_[4]:
      rospy.logerr("Something went wrong in the Server Side")
    if state_dict[state_res] == state_[3]:
      rospy.logwarn("There is a warning in the Server Side")


if __name__ == "__main__":
    mb = MoveBaseClient()
    


    
        

