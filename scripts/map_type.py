#!/usr/bin/env python
import numpy as np
import rospy
import tf
import cv2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

class MapType(object):

	def occupancy_callback(self, data):
		self.occupancy = data.data
		counter = 0
		for i in range(0, len(self.occupancy)):
			if (self.occupancy[i] == 100):
				counter += 1
		print counter, " <--counter \n"	
		#if self.flag == 1:
		if counter != 5964918:
			self.slow_down_cowboy = 1
		#	self.kraj = 1
		self.type.publish(self.slow_down_cowboy)
		print "OVDJE SAMM"
		

#	def odometry_callback(self, data):
#		i_am_speed = data.twist.twist.linear.x
#		if i_am_speed >= 0.1:
#			self.flag = 1	


	def __init__(self):
		self.flag = 0
		#self.kraj = 0
		self.slow_down_cowboy = 0
        	self.type = rospy.Publisher("/map_type", Int64, queue_size = 1) #check
        	rospy.Subscriber("/map", OccupancyGrid, self.occupancy_callback, queue_size = 1)
        	#rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
        	rospy.sleep(0.5)
        	
        	rospy.spin()

if __name__ == "__main__":
    
	rospy.init_node("map_type_node")
	try:
		mt = MapType()
	except rospy.ROSInterruptException:
		pass
