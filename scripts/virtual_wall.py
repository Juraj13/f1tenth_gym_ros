#!/usr/bin/env python
import numpy as np
import rospy
import tf
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
		self.map_no_walls = data
		self.map_width = data.info.width
		self.map_height = data.info.height
		self.map_resolution = data.info.resolution
		self.map_origin = data.info.origin.position
		self.mapload_time = data.info.map_load_time
		self.map_data = data.data
	

		# Reshape the map to 2D array.
		self.map = np.array(self.map_data).reshape(self.map_height, self.map_width)


	def pose_to_pixel(self, pose_x, pose_y):
		"""Transform pose (x,y) to pixel (x,y) in the map image. """

		pix_x = int((pose_x - self.map_origin.x) / self.map_resolution)
		pix_y = int((pose_y - self.map_origin.y) / self.map_resolution)

		return [pix_x, pix_y]

	def pixel_to_pose(self, pixel_x, pixel_y):

		pose_x = int((pixel_x * self.map_resolution) + self.map_origin.x)
		pose_y = (pixel_y * self.map_resolution) + self.map_origin.y

		return [pose_x, pose_y]



	def create_new_map(self, i):
		"""Create new map for every new wall. This function is called in initialization."""

		#Get wall corners positions(x,y)
		if i == 1: #not using 2 currently
			corner1_0 = [self.wall_positions[i][0], self.wall_positions[i][1]+self.length[i]/2]
			corner2_0 = [self.wall_positions[i][0], self.wall_positions[i][1]-self.length[i]/2]
			rr2 = 0; cc2 = 0; val2 = 0; rr3 = 0; cc3 = 0; val3 = 0;
			rr4 = 0; cc4 = 0; val4 = 0; rr5 = 0; cc5 = 0; val5 = 0;
			rr6 = 0; cc6 = 0; val6 = 0;
			rr7 = 0; cc7 = 0; val7 = 0; rr8 = 0; cc8 = 0; val8 = 0;

		else:
			corner1_0 = [self.wall_positions[i][0]+self.length[i]/2, self.wall_positions[i][1]]
			corner2_0 = [self.wall_positions[i][0]-self.length[i]/2, self.wall_positions[i][1]]

			corner_add1 = [self.wall_positions[i][2] + 3, self.wall_positions[i][3]]
			corner_add2 = [self.wall_positions[i][2], self.wall_positions[i][3]]
			corner_add1_pix =self.pose_to_pixel(corner_add1[0], corner_add1[1])
			corner_add2_pix =self.pose_to_pixel(corner_add2[0], corner_add2[1])
			rr2, cc2, val2 = line_aa(corner_add1_pix[0], corner_add1_pix[1], corner_add2_pix[0], corner_add2_pix[1])

			corner_add11 = [self.wall_positions[i][4], self.wall_positions[i][5]-1]
			corner_add22 = [self.wall_positions[i][4], self.wall_positions[i][5]]
			corner_add11_pix =self.pose_to_pixel(corner_add11[0], corner_add11[1])
			corner_add22_pix =self.pose_to_pixel(corner_add22[0], corner_add22[1])
			rr3, cc3, val3 = line_aa(corner_add11_pix[0], corner_add11_pix[1], corner_add22_pix[0], corner_add22_pix[1])

			corner_add111 = [self.wall_positions[i][6]+3, self.wall_positions[i][7]]
			corner_add222 = [self.wall_positions[i][6], self.wall_positions[i][7]]
			corner_add111_pix =self.pose_to_pixel(corner_add111[0], corner_add111[1])
			corner_add222_pix =self.pose_to_pixel(corner_add222[0], corner_add222[1])
			rr4, cc4, val4 = line_aa(corner_add111_pix[0], corner_add111_pix[1], corner_add222_pix[0], corner_add222_pix[1])

			corner_add1111 = [self.wall_positions[i][8]+3, self.wall_positions[i][9]]
			corner_add2222 = [self.wall_positions[i][8], self.wall_positions[i][9]]
			corner_add1111_pix =self.pose_to_pixel(corner_add1111[0], corner_add1111[1])
			corner_add2222_pix =self.pose_to_pixel(corner_add2222[0], corner_add2222[1])
			rr5, cc5, val5 = line_aa(corner_add1111_pix[0], corner_add1111_pix[1], corner_add2222_pix[0], corner_add2222_pix[1])

			corner_add11111 = [self.wall_positions[i][10]+3, self.wall_positions[i][11]]
			corner_add22222 = [self.wall_positions[i][10], self.wall_positions[i][11]]
			corner_add11111_pix =self.pose_to_pixel(corner_add11111[0], corner_add11111[1])
			corner_add22222_pix =self.pose_to_pixel(corner_add22222[0], corner_add22222[1])
			rr6, cc6, val6 = line_aa(corner_add11111_pix[0], corner_add11111_pix[1], corner_add22222_pix[0], corner_add22222_pix[1])

			corner_add3 = [self.wall_positions[i][12]+3, self.wall_positions[i][13]]
			corner_add4 = [self.wall_positions[i][12], self.wall_positions[i][13]]
			corner_add3_pix =self.pose_to_pixel(corner_add3[0], corner_add3[1])
			corner_add4_pix =self.pose_to_pixel(corner_add4[0], corner_add4[1])
			rr7, cc7, val7 = line_aa(corner_add3_pix[0], corner_add3_pix[1], corner_add4_pix[0], corner_add4_pix[1])

			corner_add5 = [self.wall_positions[i][14]+3, self.wall_positions[i][15]]
			corner_add6 = [self.wall_positions[i][14], self.wall_positions[i][15]]
			corner_add5_pix =self.pose_to_pixel(corner_add5[0], corner_add5[1])
			corner_add6_pix =self.pose_to_pixel(corner_add6[0], corner_add6[1])
			rr8, cc8, val8 = line_aa(corner_add5_pix[0], corner_add5_pix[1], corner_add6_pix[0], corner_add6_pix[1])



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
		new_map_array[cc2, rr2] = val2 * 100
		new_map_array[cc3, rr3] = val3 * 100
		new_map_array[cc4, rr4] = val4 * 100
		new_map_array[cc5, rr5] = val5 * 100
		new_map_array[cc6, rr6] = val6 * 100
		new_map_array[cc7, rr7] = val7 * 100
		new_map_array[cc8, rr8] = val8 * 100

		# Make sure every element in new map in integer value.
		new_map_array_int = []
		for i in range(0,self.map_height):
			for j in range(0,self.map_width):
				new_map_array_int.append(new_map_array[i][j].item())

		# Make new map a tuple.
		self.reshape_tuple_map.append(tuple(new_map_array_int))


	def give_if_new_wall(self, j, corner1, corner2):
		"""Return True if robot is in the position for new wall to be generated."""

		if j == 1: #not using 2 currently
			return (self.robot_x <= self.wall_positions[j][0]) and corner1[1] >= self.robot_y >= corner2[1]
		else:
			return (self.wall_positions[j][1]  >= self.robot_y >= self.wall_positions[j][1] - 3) and corner1[0] >= self.robot_x >= corner2[0]


	def check_goal_pose (self):


		for i in range(0, len(self.goal_positions)):
			goal_pix = self.pose_to_pixel(self.goal_positions[i][0], self.goal_positions[i][1])
			value_at_goal = self.map[goal_pix[1]][goal_pix[0]]
			cnt = 0
			while value_at_goal != 0 and cnt < 50:
				goal_pix[0] = goal_pix[0]
				goal_pix[1] = goal_pix[1]-1
				cnt+=1;
				value_at_goal = self.map[goal_pix[1]][goal_pix[0]]

			self.goal_positions[i] = self.pixel_to_pose(goal_pix[0], goal_pix[1])
			goal_pix = self.pose_to_pixel(0.0, 0.0)
			value_at_goal = self.map[goal_pix[1]][goal_pix[0]]



		



	def __init__(self):
		"""
		Create subscribers, publishers.
		Follow robot position, build walls and give goal position to global planner.

		"""

		# Initialization.
		self.flag = 0
		self.num_laps = 2
		self.num_of_walls = 2
		self.wall_positions = [[-0.2, 0.8, 81.103, 59.188, 48.2, -22.0, 107.824, 71.227, 63.271, 53.450, 117.350, 76.227, 96.063, 66.165, 51.360, 49.805], [-4.108, 12.078]]# 35.919, 30.911
		self.goal_positions = [[-0.5, 1.1], [49.534, -22.666]]#41.355, 10.498 #49,980 -23,081

		self.length = [9.7, 4.3]
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

		self.check_goal_pose()

		for i in range(0, self.num_laps):
			for j in range (0, self.num_of_walls):

				# Wait for robot to pass new wall position.
		 		while not self.give_if_new_wall(j, self.corner1[j], self.corner2[j]):
		 			rospy.sleep(0.01)

		 		# Publish map with new wall.

		 		if j == 0:
		 			new_map.header.stamp = rospy.Time.now()
		 			new_map.data = self.reshape_tuple_map[j]
		 			rospy.sleep(1.8)
		 			self.pub.publish(new_map)
		 			rospy.sleep(2.5)

		 		if (j == 1):
		 			self.pub.publish(self.map_no_walls)
		 			rospy.sleep(1.5)
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
		 		
		 		self.pub2.publish(goal_pub)


		rospy.spin()




if __name__ == "__main__":

	rospy.init_node("virtual_wall_node")
	try:
		vw = VirtualWall()
	except rospy.ROSInterruptException:
		pass
