#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry



def get_distance(x1, x2, y1, y2):
	"""Calculate distance of the line conecting two points"""
	distance = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
	return distance


class GlobalRun(object):


	def odometry_callback(self, data):
		"""Get Robot postition and orientation"""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w


	def __init__(self):
		self.num_laps = 2
		self. goal = \
		 [[9.2, -6.13, -0.83, 0.55],\
		 [5.4, -23.3, 0.1, 0.05],\
		 [2, -20.5, 0.767, 0.64],\
		 [-0.1, -11, 0.8, 0.6],\
		 [-0.9, -5.2, 0.85, 0.53],\
		 [-4.4, -5.3, 0.95, 0.28],\
		 [0, 0, 0, 1]]
		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.sleep(0.5) 
		self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
		rospy.sleep(0.5) 
		goal_length = len(self.goal)
		goal_pub = PoseStamped()
		rate = rospy.Rate(50)
		for j in range(0, self.num_laps):
			i = 0
			for i in range(0, goal_length):
				print i
				distance = 2
				goal_pub.header.stamp = rospy.Time.now()
				goal_pub.header.frame_id = "map"
				goal_pub.pose.position.x = self.goal[i][0]
				goal_pub.pose.position.y = self.goal[i][1]
				goal_pub.pose.position.z = 0
				goal_pub.pose.orientation.x = 0
				goal_pub.pose.orientation.y = 0
				goal_pub.pose.orientation.z = self.goal[i][2]
				goal_pub.pose.orientation.w = self.goal[i][3]
				self.pub.publish(goal_pub)
				while (distance > 1.5) and (not rospy.is_shutdown()):
					distance = get_distance (self.robot_x, self.goal[i][0], self.robot_y, self.goal[i][1])
					rate.sleep()
		rospy.spin()


if __name__ == "__main__":

	rospy.init_node("run_around")
	try:
		gr = GlobalRun()
	except rospy.ROSInterruptException:
		pass



