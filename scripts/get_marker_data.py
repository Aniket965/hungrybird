#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray

class prettyfloat(float):
    def __repr__(self):
        return "%0.3f" % self

#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}

		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		temp_dict = {}
		poss_array = msg.poses
		for (i, pos) in enumerate(poss_array):
			temp_dict[i] = map(prettyfloat,[pos.position.x,pos.position.y,pos.position.z])
		self.whycon_marker = temp_dict
		# Printing the detected markers on terminal
		print "\n"
		print "WhyCon_marker",self.whycon_marker
		print "ArUco_marker",self.aruco_marker


	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		temp_dict = {}
		markers = msg.markers
		for (i,marker) in enumerate(markers):
			orientation = marker.pose.pose.orientation
			temp_dict[i] = map(prettyfloat,[orientation.x,orientation.y,orientation.z,orientation.w])
		self.aruco_marker = temp_dict






if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()