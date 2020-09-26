#!/usr/bin/env python
import rospy
import math
import numpy
import tf
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from f1tenth_gym_ros.cfg import RacecarConfig



def get_distance(x1, x2, y1, y2):
	"""Calculate distance of the line conecting two points"""
	distance = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
	return distance


def get_angle_rad(x1, x2, y1, y2):
	"""
	Calculate angle in radians of the line with respect to map cordinate frame.
	Line is defined with two points (x1,y1) and (x2,y2)
	"""
	angle = math.atan2(y2-y1, x2-x1)
	return angle


class PurePursuit(object):
	"""
	Pure Pursuit algorithm.

	Pure pursuit is a path tracking algorithm. It computes the 
	steering angle command that moves the robot from its current 
	position to reach some look-ahead point in front of the robot.
	"""
	
	def calculate_pose(self, path_x, path_y):
		p1 = PoseStamped()
		p1.header.frame_id = "map"
		p1.header.stamp = self.transf.getLatestCommonTime("ego_racecar/base_link", "/map")
		p1.pose.position.x = path_x
		p1.pose.position.y = path_y
		return self.transf.transformPose("ego_racecar/base_link", p1)

	def calculate_pure_pursuit(self):

		L = 0.3302
		self.speed2 = self.speed # default: 2.0
		#look-ahead distance ld based on the speed of the vehicle
		ld = self.coefficient * self.speed2 # default: 1.0
		diff_min = ld
		
		pp = self.path_position
		self.length2 = len(pp)
		rob_x = self.robot_x
		rob_y = self.robot_y
		rob_qua_x = self.robot_qua_x
		rob_qua_y = self.robot_qua_y
		rob_qua_z = self.robot_qua_z
		rob_qua_w = self.robot_qua_w

		#find position(x,y) on the path that is ld away from the robot.
		for i in range(0, self.length2):
			path_x = pp[i][0]
			path_y = pp[i][1]
			
			path_base_link = self.calculate_pose(path_x, path_y)
			if path_base_link.pose.position.x  < -0.05:
				continue
			
			dist_rp = get_distance(rob_x, path_x, rob_y, path_y)
			diff = abs(dist_rp - ld) 
			if diff < diff_min:
				diff_min = diff
				self.goal_x = path_x
				self.goal_y = path_y

		#show a point in Rviz on the Pure Pursuit goal position
		goal = PointStamped()
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = "/map"
		goal.point.x = self.goal_x
		goal.point.y = self.goal_y
		goal.point.z = 0
		self.pub2.publish(goal)

		#calculate alpha1, alpha2 to get steering angle of the robot
		#alpha1 - angle between robot and goal position in radians.
		alpha1 = get_angle_rad(rob_x, self.goal_x, rob_y, self.goal_y)
		quaternion = [rob_qua_x, rob_qua_y, rob_qua_z, rob_qua_w]
		rpy = tf.transformations.euler_from_quaternion(quaternion)
		#alpha2 - robots orientation with respect to map cordinate frame
		alpha2 = rpy[2]
		alpha = alpha1-alpha2
		self.delta = math.atan((2*L*math.sin(alpha))/ld)

		#get distance of robot and goal position
		#dist_rg = get_distance(rob_x, self.goal_x, rob_y, self.goal_y)
		#if dist_rg < 0.1:
		#	dist_rg = 0
		#increase the robot speed if the distance is less then parameter decelerate_dist.
		#if dist_rg <= self.decelerate_dist:
		#	self.speed2 = (self.speed2 * dist_rg) / self.decelerate_dist
		
		#publish speed and steering angle of the robot
		whereto = AckermannDriveStamped()
		whereto.header.stamp = rospy.Time.now()
		whereto.drive.steering_angle = self.delta
		whereto.drive.steering_angle_velocity = 0
		whereto.drive.speed = self.speed2
		whereto.drive.acceleration = 0
		whereto.drive.jerk = 0
		self.pub.publish(whereto)
			
		


	def config_callback(self, config, level):
		"""Get configuration parameters from rqt_recongifure"""

		rospy.loginfo("""Reconfiugre Request: {speed}, {coefficient}""".format(**config))
		self.speed = config.speed
		self.coefficient = config.coefficient
		self.decelerate_dist = config.decelerate_dist
		return config


	def planner_callback(self, data):
		"""
		Get path from path planner node. 
		Make a list of all the poses in the path.
		"""
		
		self.path_position = []
		self.length = len(data.poses)

		for i in range(0, self.length):
			self.path_position.append([data.poses[i].pose.position.x, data.poses[i].pose.position.y])
		self.flag = 1


	def odometry_callback(self, data):
		"""Get Robot postition and orientation"""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w


	def __init__(self):
		"""Create subscribers, publishers and servers."""
		self.flag = 0

		srv = Server(RacecarConfig, self.config_callback)
		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.sleep(0.5) 
		rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.planner_callback, queue_size = 1)
		rospy.sleep(0.5) 
		self.pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 1)
		rospy.sleep(0.5) 
		self.pub2 = rospy.Publisher("/pure_pursit_goal", PointStamped, queue_size = 1)
		self.transf = tf.TransformListener()
		rospy.sleep(0.5) 

		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			if self.flag == 1:
				self.calculate_pure_pursuit()
			rate.sleep()
		


if __name__ == "__main__":

	rospy.init_node("Pure_pursuit_node")
	try:
		pp = PurePursuit()
	except rospy.ROSInterruptException:
		pass
