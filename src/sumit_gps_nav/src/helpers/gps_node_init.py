#! /usr/bin/env python

# import necessary packages
import rospy
from sensor_msgs.msg import NavSatFix
import geonav_transform.geonav_conversions as gc
from geopy.distance import vincenty
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quartenion_from_euler

"""
used parameters from NavSatFix
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 latitude
float64 longitude
float64 altitude
"""

# class for waypoints construction 
class WayPoints:
  # stores latitude altitude and longitude 
  def __init__(self, lat, lon, alt=0):
    self.lon = lon
    self.lat = lat
    self.alt = alt

  #printing function
  def print_(self):
    return "[lat:{}, lon:{}, alt:{}]".format(self.lat, 
    self.lon, self.alt)

# GPS class -> stores waypoints and sets gps origin and destination
class GpsNode:
  def __init__(self):
    # initialize node
    rospy.init_node("gps_sub_node")

    # rate and exit init
    self.rate = rospy.Rate(10)
    self.ctrl_c = False

    # Extract current gps location
    self.current_wp = WayPoints(0, 0)
    self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, callback=self.gps_callback)  

    # destination pose array [x,y, yaw]
    self.dest_wp = None    


  # call back registers current waypoint
  def gps_callback(self, msg):
    self.current_wp.lat, self.current_wp.lon, self.current_wp.alt = self.noise_rem(msg.latitude, msg.longitude, msg.altitude)
    

  # returns pose
  def getPose_from_lat_long(self):
    delta_x, delta_y = gc.ll2xy(self.dest_wp.lat, self.dest_wp.lon, self.current_wp.lat, self.current_wp.lon)

    # calculate angle
    theta = atan2(delta_y, delta_x)

    # calculate quaternion 
    quart = quartenion_from_euler(0, 0, theta)

    # fill the pose
    pose = Pose()
    pose.position.x = delta_x
    pose.position.y = delta_y
    pose.position.z = 0
    
    q = Quaternion()
    q.x = quart[0]
    q.y = quart[1]
    q.z = quart[2]
    q.w = quart[3]

    pose.orientation = q

    return pose

   # remove noise from gps data
  def noise_rem(self, lat, lon, alt):
    return round(lat, 5), round(lon, 5), round(alt, 1)

  # function for setting destination way point
  def dest_init(self, lat, lon, alt=0):
    # create destination waypoint
    lat, lon, alt = self.noise_rem(lat, lon, alt)
    self.dest_wp = WayPoints(lat, lon, alt)

  def waypoint_distance(self, unit="m"):
    origin = (self.current_wp.lon, self.current_wp.lat)
    target = (self.destination_wp.longitude, self.destination_wp.latitude)

    # make units lower case
    unit = unit.lower()

    # use vincenty method
    if unit == "km" or unit == "kilometers":
      distance = vincenty(origin, target).kilometers

    elif unit == "meters" or unit == "m":
      distance = vincenty(origin, target).meters

    else:
      distance = vincenty(origin, target).miles
        
    # print
    rospy.loginfo("{} [{}] to destination".format(distance, unit)) 
    return distance
