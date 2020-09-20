#!/usr/bin/env python
import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf import TransformListener


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

	def get_tf_transform(self, tf1, tf2):

		self.listener.waitForTransform(tf1, tf2, rospy.Time(), rospy.Duration(4.0))
		(trans,rot) = self.listener.lookupTransform(tf1, tf2, rospy.Time(0))
		print 'transformation between tf1 and tf2 detected'
		print 'translation vector: (',trans[0],',',trans[1],',',trans[2],')'
		print 'rotation : x=',rot[0],' y=',rot[1],' z=',rot[2], 'w=',rot[3]
		return trans, rot

	def __init__(self):

		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.sleep(0.5) 
		rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size = 1)
		rospy.sleep(0.5) 
		self.listener = tf.TransformListener()
		trans_p1, rot_p1 = self.get_tf_transform('/ego_racecar/p1', '/ego_racecar/base_link')
		trans_p2, rot_p2 = self.get_tf_transform('/ego_racecar/p2', '/ego_racecar/base_link')
		rospy.spin()






if __name__ == "__main__":

	rospy.init_node("Virtual_wall_node")
	try:
		vw = VirtalWall()
	except rospy.ROSInterruptException:
		pass