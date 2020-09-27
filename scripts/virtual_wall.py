#!/usr/bin/env python
import numpy as np
import rospy
import tf
import cv2
from nav_msgs.msg import Odometry, Path, OccupancyGrid
#from tf import TransformListener
from geometry_msgs.msg import PointStamped, PoseStamped
from skimage.draw import line_aa



class VirtualWall(object):
	"""
	Virtual wall node 

	Generates imaginary wall behind the car so global planner could plan entire lap at once. 
	Two walls are built based on robots location, one in the start and other in the middle of the lap.
	"""

	def odometry_callback(self, data):
		"""Get Robot postition and orientation."""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w


	def map_callback(self, data):
		"""Get map information."""

		self.map_width = data.info.width
		self.map_height = data.info.height
		self.map_resolution = data.info.resolution
		self.map_origin = data.info.origin.position
		self.mapload_time = data.info.map_load_time
		self.map_data = data.data

		# Reshape the map to 2D array.
		self.map = np.array(self.map_data).reshape(self.map_width, self.map_height)


	def pose_to_pixel(self, pose_x, pose_y):
		"""Transform pose (x,y) to pixel (x,y) in the map image. """

		pix_x = int((pose_x - self.map_origin.x) / self.map_resolution)
		pix_y = int((pose_y - self.map_origin.y) / self.map_resolution)

		return [pix_x, pix_y]


	def build_wall(self, corner1_pix, corner2_pix):
		"""Build imaginary wall defined by two pixels (corner1_pix, corner2_pix)"""

		# Define new map same as the one form /map topic
		new_map = OccupancyGrid()
		new_map.header.stamp = rospy.Time.now()
		new_map.info.width = self.map_width
		new_map.info.height = self.map_height
		new_map.info.resolution = self.map_resolution
		new_map.info.origin.position = self.map_origin
		new_map.info.map_load_time = self.mapload_time
		new_map_array = np.array(self.map_data).reshape(self.map_width, self.map_height)

		# Generate a line through two pixels. Get row, column and value (0-1) od each pixel in the line.
		rr, cc, val = line_aa(corner1_pix[0], corner1_pix[1], corner2_pix[0], corner2_pix[1])

		# Scale the pixel values to range 0-100.
		new_map_array[cc, rr] = val * 100

		# Change every pixels value to int and reshape map to list.
		new_map_array_int = []
		for i in range(0,self.map_width):
			for j in range(0,self.map_height):
				new_map_array_int.append(new_map_array[i][j].item())

		# Make new map a tuple and publish it
		reshape_tuple = tuple(new_map_array_int)
		new_map.data = reshape_tuple
		self.pub.publish(new_map)

		# Flag set to 1 if the map
		self.flag = 1

	def give_if_new_wall(self, j):
		"""Return True if robot is in the position for new wall to be generated"""

		if j == 0:
			return (self.robot_x >= self.wall_positions[j][0]) and self.corner1[1] >= self.robot_y >= self.corner2[1]
		else:
			return self.robot_y <= self.wall_positions[j][1] and self.corner1[0] >= self.robot_x >= self.corner2[0]



	def __init__(self):
		"""
		Create subscribers, publishers.
		Follow robot position, build walls and give goal position to global planner.
		
		"""
		# Initialization
		self.flag = 0
		self.domagoj_sux = 1
		self.num_laps = 2
		self.num_of_walls = 2
		self.wall_positions = [[-0.5, 0], [7.31, -10.41]] #5.14, -24.04
		self.goal_positions = [[-2.5, 0], [7.31, -10.00]]
		self.length = [4.1, 8]

		# Create subscribers
		rospy.Subscriber("/odom", Odometry,self.odometry_callback, queue_size = 1)
		rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size = 1)
		rospy.sleep(0.5)

		# Create publishers
		self.pub = rospy.Publisher("/map2",OccupancyGrid, queue_size = 1)
		self.pub2 = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
		rospy.sleep(0.5)

		for i in range(0, self.num_laps):
			for j in range (0, self.num_of_walls):

				# Find wall corners
				if j == 0:
					self.corner1 = [self.wall_positions[j][0], self.wall_positions[j][1]+self.length[j]/2]
					self.corner2 = [self.wall_positions[j][0], self.wall_positions[j][1]-self.length[j]/2]
				if j == 1:
					self.corner1 = [self.wall_positions[j][0]+self.length[j]/2, self.wall_positions[j][1]]
					self.corner2 = [self.wall_positions[j][0]-self.length[j]/2, self.wall_positions[j][1]]

				# Find wall corner pixels
				corner1_pix = self.pose_to_pixel(self.corner1[0], self.corner1[1])
				corner2_pix =self.pose_to_pixel(self.corner2[0], self.corner2[1])

				# Check if robot has passed the wall
				while not self.give_if_new_wall(j):

					rospy.sleep(0.2)

				self.build_wall(corner1_pix, corner2_pix)

				# Wait for map to be published
				while not self.flag == 1:
					rospy.sleep(0.2)

				self.flag = 0

				# Define and publish goal pose for global planner
				goal_pub = PoseStamped()
				goal_pub.header.stamp = rospy.Time.now()
				goal_pub.header.frame_id = "map"
				goal_pub.pose.position.x = self.goal_positions[j][0]
				goal_pub.pose.position.y = self.goal_positions[j][1]
				goal_pub.pose.position.z = 0
				goal_pub.pose.orientation.x = 0
				goal_pub.pose.orientation.y = 0
				goal_pub.pose.orientation.z = 0
				goal_pub.pose.orientation.w = 1
				rospy.sleep(0.2)
				self.pub2.publish(goal_pub)

		rospy.spin()


if __name__ == "__main__":

	rospy.init_node("virtual_wall_node")
	try:
		vw = VirtualWall()
	except rospy.ROSInterruptException:
		pass
