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
		print "map_height: ", self.map_height, "  map_width: ", self.map_width, " map_resolution: ", self.map_resolution, "\n"
		print "map_origin_x: ", self.map_origin.x, " map_origin_y: ", self.map_origin.y

		# Reshape the map to 2D array.
		self.map = np.array(self.map_data).reshape(self.map_width, self.map_height)


	def pose_to_pixel(self, pose_x, pose_y):
		"""Transform pose (x,y) to pixel (x,y) in the map image. """

		pix_x = int((pose_x - self.map_origin.x) / self.map_resolution)
		pix_y = int((pose_y - self.map_origin.y) / self.map_resolution)

		return [pix_x, pix_y]



	def create_new_map(self, i):
		"""Create new map for every new wall. This function is called in initialization."""

		#Get wall corners positions(x,y)
		if i == 2: #not using 2 currently
			corner1_0 = [self.wall_positions[i][0], self.wall_positions[i][1]+self.length[i]/2]
			corner2_0 = [self.wall_positions[i][0], self.wall_positions[i][1]-self.length[i]/2]
		else:
			corner1_0 = [self.wall_positions[i][0]+self.length[i]/2, self.wall_positions[i][1]]
			corner2_0 = [self.wall_positions[i][0]-self.length[i]/2, self.wall_positions[i][1]]

		self.corner1.append(corner1_0)
		self.corner2.append(corner2_0)

		# Get pixel(x,y) of two wall corners.
		corner1_pix = self.pose_to_pixel(self.corner1[i][0], self.corner1[i][1])
		corner2_pix =self.pose_to_pixel(self.corner2[i][0], self.corner2[i][1])

		# Generate line in between two wall corners.
		rr, cc, val = line_aa(corner1_pix[0], corner1_pix[1], corner2_pix[0], corner2_pix[1])

		# Add wall to new map.
		new_map_array = np.array(self.map_data).reshape(self.map_height, self.map_width)
		new_map_array[cc, rr] = val * 100

		# Make sure every element in new map in integer value.
		new_map_array_int = []
		for i in range(0,self.map_height):
			for j in range(0,self.map_width):
				new_map_array_int.append(new_map_array[i][j].item())

		# Make new map a tuple.
		self.reshape_tuple_map.append(tuple(new_map_array_int))


	def give_if_new_wall(self, j, corner1, corner2):
		"""Return True if robot is in the position for new wall to be generated."""

		if j == 2: #not using 2 currently
			return (self.robot_x <= self.wall_positions[j][0]) and corner1[1] >= self.robot_y >= corner2[1]
		else:
			return (self.wall_positions[j][1]  >= self.robot_y >= self.wall_positions[j][1] - 3) and corner1[0] >= self.robot_x >= corner2[0]



	def __init__(self):
		"""
		Create subscribers, publishers.
		Follow robot position, build walls and give goal position to global planner.

		"""

		# Initialization.
		self.flag = 0
		self.num_laps = 2
		self.num_of_walls = 2
		self.wall_positions = [[-0.9, 2.037], [35.919, 30.911]] #5.14, -24.04
		self.goal_positions = [[41.355, 10.498], [51.76, -25.96]]
		self.length = [9.7, 6]
		self.reshape_tuple_map = []
		self.corner1 = []
		self.corner2 = []


		# Create subscribers.
		rospy.Subscriber("/odom", Odometry,self.odometry_callback, queue_size = 1)
		rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size = 1)
		rospy.sleep(0.5)

		# Create publishers.
		self.pub = rospy.Publisher("/map2",OccupancyGrid, queue_size = 1)
		self.pub2 = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
		rospy.sleep(0.5)

		# Initialize new map.
		new_map = OccupancyGrid()
		new_map.header.stamp = rospy.Time.now()
		new_map.info.width = self.map_width
		new_map.info.height = self.map_height
		new_map.info.resolution = self.map_resolution
		new_map.info.origin.position = self.map_origin
		new_map.info.map_load_time = self.mapload_time

		# Create a map for every wall used in this race.
		for i in range(0, self.num_of_walls):
			self.create_new_map(i)

		for i in range(0, self.num_laps):
			for j in range (0, self.num_of_walls):

				# Wait for robot to pass new wall position.
		 		while not self.give_if_new_wall(j, self.corner1[j], self.corner2[j]):
		 			rospy.sleep(0.01)

		 		# Publish map with new wall.
		 		new_map.header.stamp = rospy.Time.now()
		 		new_map.data = self.reshape_tuple_map[j]
		 		self.pub.publish(new_map)

		 		# Define and publish goal pose for global planner.
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
		 		rospy.sleep(0.1)
		 		self.pub2.publish(goal_pub)

		rospy.spin()




if __name__ == "__main__":

	rospy.init_node("virtual_wall_node")
	try:
		vw = VirtualWall()
	except rospy.ROSInterruptException:
		pass
