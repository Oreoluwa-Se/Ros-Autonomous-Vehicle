#! /usr/bin/env python

# import necessary packages
import rospy
from sensor_msgs.msg import NavSatFix
from geopy.distance import vincenty
from geometry_msgs.msg import Vector3

"""
output from the NavSatFix
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
"""

# class for way point construction
class Waypoints:
	def __init__(self, longitude=0, latitude=0, altitude=0):
		self.longitude = longitude
		self.latitude = latitude
		self.altitude = altitude

	def print(self):
		return "[{},{},{}]".format(self.longitude, self.latitude, self.altitude)

# class to subscribe to GPS data
class GPSNode:
	def __init__(self, topic="/fix", curr_long=0, curr_lat=0, curr_alt=0):
		# initialize node
		rospy.init_node("gps_sub_node")

		# current way point
		self.current_wp = Waypoints(curr_long, curr_lat, curr_alt) 	

        # initialize current 
        self.destination_wp = None

		# subscribe to the topic
		self.gps_sub = rospy.Subscriber(topic, NavSatFix, self.gps_callback) 


	# updates the current way point
	def gps_callback(self, msg):
		self.current_wp.longitude, self.current_wp.latitude, self.current_wp.altitude = self.noise_rem(msg.longitude, msg.latitude, msg.altitude) 

	# calculate distance between waypoints
	def waypoint_dist(self, unit="km"):
		"""
			output in kilometers
		"""
		origin = (self.current_wp.longitude, self.current_wp.latitude)
		target = (self.destination_wp.longitude, self.destination_wp.latitude)

		# make units lower case
		unit = unit.lower()

		# use vincenty method
		if unit == "km" or unit == "kilometers":
			return vincenty(origin, target).kilometers

		elif unit == "meters" or unit == "m":
			return vincenty(origin, target).meters

		else:
			return vincenty(origin, target).miles

	# xyz direction betweenw ay points
	def xyz_dir(self, waypoints):
		distance = Vector3()

		distance.x = waypoints.longitude - self.current_wp.longitude
		distance.y = waypoints.latitude - self.current_wp.latitude
		distance.z = waypoints.altitude - self.current_wp.altitude

		return distance

	# used to initialize destination way point
	def set_destination(self, tag):
		locations = {}

		locations[1] = [43.70139, -79.6919794, 185]
		locations[2] = [43.826651, -79.184303, 138]
		locations[3] = [45.422527, -75.6833904, 58]

		# ensures valid location entered
		if tag not in locations.keys():
			tag = input("Location not stored. Enter new location from {}".format(locations.keys()))

		# initialize 
		self.destination_wp = Waypoints(*locations[tag])
		

	# remove noise from gps data
	def noise_rem(self, longitude, latitude, altitude):
		return round(longitude, 5), round(latitude, 5), round(altitude, 1) 

# run class
if __name__ == "__main__":
	# instantiate and spin
	GPSNode()
	rospy.spin()