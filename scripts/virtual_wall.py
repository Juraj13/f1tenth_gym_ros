#!/usr/bin/env python
import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry


class VirtalWall(object):


	def odometry_callback(self, data):
		"""Get Robot postition and orientation"""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w

	def map_callback(self, data):

		self.map_width = data.info.width
		self.map_height = data.info.height
		self.map_resolution = data.info.resolution
		self.map_origin = data.info.origin.position
		self.load_time = data.info.map_load_time
		self.map = data.data
		self.map = np.array(self.map).reshape(self.map_width, self.map_height)

		# calculate robots position in pixels
		self.robot_pix_x, self.robot_pix_y = self.position_to_pixels(self.robot_x, self.robot_y)

	def position_to_pixels(self, pose_x, pose_y):
		pix_x = int((pose_x - self.map_origin.x) / self.map_resolution)
		pix_y = int((pose_y - self.map_origin.y) / self.map_resolution)
		return pix_x, pix_y


	def __init__(self):

		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.sleep(0.5) 
		rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size = 1)
		rospy.sleep(0.5) 
		rospy.spin()






if __name__ == "__main__":

	rospy.init_node("Virtual_wall_node")
	try:
		vw = VirtalWall()
	except rospy.ROSInterruptException:
		pass