#!/usr/bin/env python
import rospy
import math
import tf
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped



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

	def calculate_pure_pursuit(self):

		L = 0.3302
		diff_min = 0.2
		speed = 2
		#look-ahead distance ld based on the speed of the vehicle
		ld = 0.5 * speed
		length = len(self.path_position)

		#find position(x,y) on the path that is ld away from the robot.
		for i in range(0, length):
			path_x = self.path_position[i][0]
			path_y = self.path_position[i][1]
			dist_rp = get_distance(self.robot_x, path_x, self.robot_y, path_y)
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
		alpha1 = get_angle_rad(self.robot_x, self.goal_x, self.robot_y, self.goal_y)
		quaternion = [self.robot_qua_x, self.robot_qua_y, self.robot_qua_z, self.robot_qua_w]
		rpy = tf.transformations.euler_from_quaternion(quaternion)
		#alpha2 - robots orientation with respect to map cordinate frame
		alpha2 = rpy[2]
		alpha = alpha1-alpha2
		delta = math.atan((2*L*math.sin(alpha))/ld)

		#get distance of robot and goal position
		dist_rg = get_distance(self.robot_x, self.goal_x, self.robot_y, self.goal_y)
		if dist_rg < 0.05:
			dist_rg = 0
		#decrease the robot speed if the distance is less then 0.5.
		if dist_rg <= 0.5:
			speed = (speed * dist_rg) / 0.5

		#publish speed and steering angle of the robot
		whereto = AckermannDriveStamped()
		whereto.header.stamp = rospy.Time.now()
		whereto.drive.steering_angle = delta
		whereto.drive.steering_angle_velocity = 1
		whereto.drive.speed = speed
		whereto.drive.acceleration = 1
		whereto.drive.jerk = 1
		self.pub.publish(whereto)




	def planner_callback(self, data):
		"""
		Get path from path planner node. 
		Make a list of all the poses in the path.
		"""

		self.path_position = []
		self.length = len(data.poses)

		for i in range(0,self.length):
			self.path_position.append([data.poses[i].pose.position.x, data.poses[i].pose.position.y])


	def odometry_callback(self, data):
		"""Get Robot postition and orientation"""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w


	def __init__(self):
		"""Create subscribers and publishers."""

		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.sleep(0.5) 
		rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.planner_callback, queue_size = 1)
		rospy.sleep(0.5) 
		self.pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 1)
		rospy.sleep(0.5) 
		self.pub2 = rospy.Publisher("/pure_pursit_goal", PointStamped, queue_size = 1)
		rospy.sleep(0.5) 

		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			self.calculate_pure_pursuit()
			rate.sleep()
		rospy.spin()


if __name__ == "__main__":

	rospy.init_node("pure_pursuit_node")
	try:
		pp = PurePursuit()
	except rospy.ROSInterruptException:
		pass