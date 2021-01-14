#!/usr/bin/env python
import rospy
import math
import tf
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Twist


class Controller(object):
	def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
		if omega == 0 or v == 0:
			return 0
		radius = v / omega
		return math.atan(wheelbase / radius)


	def control(self):
		command = AckermannDriveStamped()
		command.header.stamp = rospy.Time.now()
		command.drive.steering_angle = self.convert_trans_rot_vel_to_steering_angle(self.v, self.w, 0.3302)
		command.drive.speed = self.v

		self.pub.publish(command)


	def cmd_callback(self, msg):
		self.v = msg.linear.x/2
		self.w = msg.angular.z


	def __init__(self):
		self.v = 0
		self.w = 0
		"""Create subscribers and publishers."""
		rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size = 1)
		rospy.sleep(0.5) 
 
		self.pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 1)
		rospy.sleep(0.5) 

		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.control()
			rate.sleep()
		rospy.spin()




if __name__ == "__main__":

	rospy.init_node("controller_node")
	try:
		pp = Controller()


	except rospy.ROSInterruptException:
		pass
