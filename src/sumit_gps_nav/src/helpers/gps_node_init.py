#! /usr/bin/env python

# import necessary packages
import rospy
from sensor_msgs.msg import NavSatFix
import geonav_transform.geonav_conversions as gc
from geopy.distance import vincenty

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
    
  # extracts x, y location - between current way point and origin
  def get_xy_from_lat_long(self):
    xg2, yg2 = gc.ll2xy(self.current_wp.lat, self.current_wp.lon,self.dest_wp.lat,self.dest_wp.lon)
    return xg2, yg2

   # remove noise from gps data
  def noise_rem(self, lat, lon, alt):
    return round(lat, 5), round(lon, 5), round(alt, 1)

  # function for setting destination way point
  def dest_init(self, lat, lon, alt=0):
    # create destination waypoint
    lat, lon, alt = self.noise_rem(lat, lon, alt)
    self.dest_wp = WayPoints(lat, lon, alt)

  def waypoint_distance(self, units="m"):
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

  # function for publishing origin to navsatfix
  def navsatfix_pub(self, loc, wp):
    to_pub = NavSatFix()

    # messages to be published
    to_pub.header.frame_id   = rospy.get_param("~frame_id", "map")
    to_pub.header.stamp.secs = rospy.get_time()
    to_pub.latitude          = wp.lat
    to_pub.longitude         = wp.lon
    to_pub.altitude          = wp.alt

    # publish
    while not self.ctrl_c:
      connect = self.nav_sat_pub.get_num_connections()
      # ensure we are connected
      if connect > 0:
        self.nav_sat_pub.publish(to_pub)
        rospy.loginfo("{} waypoints sent to /gps/fix topic".format(loc))
        break
      else:
        self.rate.sleep()
