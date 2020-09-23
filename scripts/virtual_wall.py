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

	def odometry_callback(self, data):
		"""Get Robot postition and orientation"""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w

	def costmap_callback(self, data):
		self.costmap_width = data.info.width
		self.costmap_height = data.info.height
		self.costmap_resolution = data.info.resolution
		self.costmap_origin = data.info.origin.position
		self.mapload_time = data.info.map_load_time
		self.costmap_data = data.data
		self.costmap = np.array(self.costmap_data).reshape(self.costmap_width, self.costmap_height)

	def planner_callback(self, data):
		"""
		Get path from path planner node. 
		Make a list of all the poses in the path.
		"""
		if self.flag2 != 0:
			self.path_positions_again.append(data.poses)
			self.i = self.i+1
			self.flag2  = self.flag2-1

	def pose_to_pixel(self, pose_x, pose_y):

		pix_x = int((pose_x - self.costmap_origin.x) / self.costmap_resolution)
		pix_y = int((pose_y - self.costmap_origin.y) / self.costmap_resolution)
		return pix_x, pix_y



	def calculate_pose(self):
		t = self.tf_listener_.getLatestCommonTime("/base_link", "/map")
		p1 = PoseStamped()
		p1.header.frame_id = "map"
		p1.header.stamp = t
		p1.pose.position.x = self.wall_pose_x
		return self.tf_listener_.transformPose("/base_link", p1)
		


	def build_wall(self):

		new_map = OccupancyGrid()
		new_map.header.stamp = rospy.Time.now()
		new_map.info.width = self.costmap_width
		new_map.info.height = self.costmap_height
		new_map.info.resolution = self.costmap_resolution
		new_map.info.origin.position = self.costmap_origin
		new_map.info.map_load_time = self.mapload_time

		new_map_array = np.array(self.costmap_data).reshape(self.costmap_width, self.costmap_height)

		self.r1 = self.v[1]+self.s/2
		self.r2 = self.v[1]-self.s/2
		r1_x, r1_y = self.pose_to_pixel(self.v[0], self.r1)
		r2_x, r2_y = self.pose_to_pixel(self.v[0], self.r2)


		rr, cc, val = line_aa(r1_x, r1_y, r2_x, r2_y)


		new_map_array[cc, rr] =  val * 100
		new_map_array = new_map_array.astype(np.int8)
		new_map_array_int = []
		for i in range(0,self.costmap_width):
			for j in range(0,self.costmap_height):
				new_map_array_int.append(new_map_array[i][j].item())
		reshape_tuple = tuple(new_map_array_int)
		new_map.data = reshape_tuple
		self.pub.publish(new_map)






	def __init__(self):
		# neka inicijalizacija
		self.i = 0
		self.flag2 = 10
		self.path_positions_again =[]
		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.Subscriber("/map", OccupancyGrid, self.costmap_callback, queue_size = 1)
		rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.planner_callback, queue_size = 1)
		rospy.sleep(0.5)
		self.pub = rospy.Publisher("/map2",OccupancyGrid, queue_size = 1)
#		self.tf = TransformListener()
		self.pub2 = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
		self.pub3 = rospy.Publisher ("/move_base/TebLocalPlannerROS/global_plan", Path, queue_size = 1)
		rospy.sleep(0.5)
#		trans, rot  = self.tf.lookupTransform('/map', 'ego_racecar/goal_link', rospy.Time(0))
		rate = rospy.Rate(5)
		self.v = np.array([-0.1, 0])
		self.s = 4.1
		self.build_wall()
		rospy.sleep(2)
		goal_pub = PoseStamped()
		self.wall_pose_x = -0.2
		goal_pub.header.stamp = rospy.Time.now()
		goal_pub.header.frame_id = "map"
		goal_pub.pose.position.x = self.wall_pose_x
		goal_pub.pose.position.y = 0
		goal_pub.pose.position.z = 0
		goal_pub.pose.orientation.x = 0
		goal_pub.pose.orientation.y = 0
		goal_pub.pose.orientation.z = 0
		goal_pub.pose.orientation.w = 1
		self.pub2.publish(goal_pub)
		rate1 = rospy.Rate(5)
		rate = rospy.Rate (50)
		self.flag = 1
		while not rospy.is_shutdown() and self.flag == 1:
			if -0.1 < self.robot_x < 0 and self.r2 < self.robot_y < self.r1:
				goal_pub.header.stamp = rospy.Time.now()
				self.pub2.publish(goal_pub)
				new_path = Path()
				new_path.header.stamp = rospy.Time.now()
				new_path.header.frame_id = "/map"
				self.flag2 = 10
				i = 0
				while self.flag2 != 0 :
					new_path.poses = self.path_positions_again[i]
					self.pub3.publish(new_path)
					self.flag2 = self.flag2-1
					i = i+1
					rate1.sleep()

				self.flag = 0
			rate.sleep()




		rospy.spin()


if __name__ == "__main__":

	rospy.init_node("virtual_wall_node")
	try:
		vw = VirtualWall()
	except rospy.ROSInterruptException:
		pass