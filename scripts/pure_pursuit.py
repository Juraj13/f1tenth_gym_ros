#!/usr/bin/env python
import rospy
import math
import numpy
import tf
from scipy import optimize
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Twist
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from fer_rari.cfg import RacecarConfig
from scipy.interpolate import interp1d
from std_msgs.msg import Int64



def get_distance(x1, y1, x2, y2):
	"""Calculate distance of the line conecting two points"""
	distance = numpy.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
	return distance


def get_angle_rad(x1, x2, y1, y2):
	"""
	Calculate angle in radians of the line with respect to map cordinate frame.
	Line is defined with two points (x1,y1) and (x2,y2)
	"""
	angle = math.atan2(y2-y1, x2-x1)
	return angle

def circle_func(c, x, y):
	""" 
	Calculate the algebraic distance between the data points 
	and the mean circle centered at c=(xc, yc) 
	"""
	Ri = get_distance(x, y, *c)
	return Ri - Ri.mean()
	



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

	def check_yourself(self):

		if ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 1 and self.mode%2 == 0):
			print "uso1"
			self.mode += 1
			self.zastavica = 0
		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and (self.zastavica == 0)):
			print "uso3"
			self.zastavica = 1
			self.cnt += 2
			self.mode += 1
			self.count_dooke += 1
			if self.count_dooke == 4:
				self.count_dooke = 0
			if self.cnt == 8:
				self.cnt = 0

	def b4_U_rack_yoself(self):
			
			
		if (self.modul == 1 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1): 
			#startna ravnina
			print ("ciljna ravnina\n")
			self.ld = 1.5
			self.v = 8
			self.turn = 1
			self.count_dooke = 0
			self.cnt = 0
			#self.count_dooke += 1
			self.zastavica = 0
			self.hrv_zastava = 1
		
		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 1):
			#prvi zavoj
			print ("prvi zavoj")
			self.v = 6.8
			self.ld = 1.2
			self.zastavica = 2
			self.count_dooke += 1
			self.hrv_zastava = 2

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 2 and self.turn == 1):
			#prvi zavoj
			print ("prvi zavoj v2")
			self.v = 3.8
			self.ld = 1.2
			self.modul = 2
			self.zastavica = 1
			self.hrv_zastava = 2

		elif (self.modul == 2 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			#ravnina nakon prvog zavoja
			print ("ravnina nakon prvog zavoja")
			self.v = 9
			self.ld = 1.5
			self.count_dooke += 1
			self.turn = 2
			self.cnt += 2
			self.zastavica = 0
			self.hrv_zastava = 1
		
		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 2):
			#drugi zavoj
			print ("drugi zavoj")
			self.v = 6.9
			self.ld = 1
			self.modul = 3
			self.zastavica = 1
			self.hrv_zastava = 1

		elif (self.modul == 3 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print("ravnina nakon drugog zavoja i 3., 4., 5. i 6. zavoj")

			self.v = 8.5
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 3
			self.hrv_zastava = 3
			

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 3):
			print "sedmi zavoj\n"
			self.v = 7.0
			self.ld = 1
			self.modul = 4

			self.zastavica = 1
			self.hrv_zastava = 1

		elif (self.modul == 4 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "ravnina nakon sedmog zavoja"
			self.v = 8
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 4
			self.hrv_zastava = 1

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 4):
			print "osmi zavoj"
			self.v = 8.5
			self.ld = 1
			self.modul = 5
			self.zastavica = 1
			self.hrv_zastava = 1

		elif ( self.modul == 5 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "ravnina nakon osmog zavoja"
			self.v = 9.1
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 5
			self.hrv_zastava = 1

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 5):
			print "deveti zavoj (skroz gore U zavoj)"
			self.v = 7.0
			self.ld = 1
			self.modul = 6
			self.zastavica = 2
			self.count_dooke += 1
			self.hrv_zastava = 2

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 2 and self.turn == 5):
			print "deveti zavoj (skroz gore U zavoj)"
			self.v = 4.1
			self.ld = 1
			self.modul = 6
			self.zastavica = 1
			self.hrv_zastava = 2

		elif (self.modul == 6 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "big boi ravnina"
			self.v = 15
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 6
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 6):
			print "zavoj nakon big boi ravnine 1"
			self.v = 13.5
			self.ld = 1
			#self.modul = 7
			self.zastavica = 2
			self.count_dooke += 1
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 2 and self.turn == 6):
			print "zavoj nakon big boi ravnine 2"
			self.v = 12
			self.ld = 1
			#self.modul = 7
			self.zastavica = 3
			self.count_dooke += 1
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 3 and self.turn == 6):
			print "zavoj nakon big boi ravnine 3"
			self.v = 10.5
			self.ld = 1
			#self.modul = 7
			self.zastavica = 6
			self.count_dooke += 1
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 6 and self.turn == 6):
			print "zavoj nakon big boi ravnine 4"
			self.v = 9
			self.ld = 1
			#self.modul = 7
			self.zastavica = 4
			self.count_dooke += 1
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 4 and self.turn == 6):
			print "zavoj nakon big boi ravnine 5"
			self.v = 7.5
			self.ld = 1
			#self.modul = 7
			self.zastavica = 7
			self.count_dooke += 1
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 7 and self.turn == 6):
			print "zavoj nakon big boi ravnine 5"
			self.v = 6.4
			self.ld = 1
			#self.modul = 7
			self.zastavica = 5
			self.count_dooke += 1
			self.hrv_zastava = 4

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 5 and self.turn == 6):
			print "zavoj nakon big boi ravnine 6"
			self.v = 5
			self.ld = 1
			self.modul = 7
			self.zastavica = 1
			self.hrv_zastava = 2

		elif (self.modul == 7 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "mini ravnina nakon tog zavoja"
			self.v = 7
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 7
			self.hrv_zastava = 1

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 7):
			print "dva zavoja (kvazi U)"
			self.v = 5
			self.ld = 1
			self.modul = 8
			self.zastavica = 1
			self.hrv_zastava = 2

		elif (self.modul == 8 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "mini ravnina nakon tog zavoja"
			self.v = 6.9
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 8
			self.hrv_zastava = 1

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 8):
			print "U turn"
			self.v = 5.0
			self.ld = 1.2
			self.modul = 9
			self.zastavica = 1
			self.hrv_zastava = 2

		elif (self.modul == 9 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "duga ravnina prije zadnja 2 zavoja"
			self.v = 8.2
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 9
			self.hrv_zastava = 1

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 9):
			print "predzadnji zavoj"
			self.v = 6.9
			self.ld = 1
			self.modul = 10
			self.zastavica = 1
			self.hrv_zastava = 2

		elif (self.modul == 10 and (get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) >= self.circle_of_life[self.count_dooke]) and self.zastavica == 1):
			print "zadnja ravnina"
			self.v = 8.5
			self.ld = 1.5
			self.count_dooke += 1
			self.cnt += 2
			self.zastavica = 0
			self.turn = 10
			self.hrv_zastava = 1

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 0 and self.turn == 10):
			print "zadnji zavoj"
			self.v = 5
			self.ld = 1
			self.zastavica = 2
			self.modul = 1
			self.count_dooke += 1
			self.hrv_zastava = 2

		elif ((get_distance(self.robot_x, self.robot_y, self.turning_points[self.cnt], self.turning_points[self.cnt+1]) <= self.circle_of_life[self.count_dooke]) and self.zastavica == 2 and self.turn == 10):
			print "zadnji zavoj"
			self.v = 5
			self.ld = 1
			self.zastavica = 1
			self.modul = 1
			self.hrv_zastava = 2

	def velocity_controller(self, pp):
		# c_est = 0.0, -10.0 # somewhere in the middle of the map
		# x = [p[0] for p in pp]
		# y = [p[1] for p in pp]
		# #plt.plot(x, y)
		# #plt.show()
		# center, ier = optimize.leastsq(circle_func, c_est, args=(x,y))
		# xc, yc = center # fitted circle center
		# Ri = get_distance(x, y, *center)
		# R = Ri.mean() # fitted circle radius
		
		# # calculate angle between first and last point on local path
		# first_point = pp[0]-center
		# first_point = first_point/numpy.linalg.norm(first_point)
		# last_point = pp[-1]-center
		# last_point = last_point/numpy.linalg.norm(last_point)
		
		# dot_product = numpy.dot(last_point,first_point)
		# angle = numpy.arccos(dot_product)
		# angle = angle*180.0/math.pi
		

		# self.check_yourself()prije
		
		# if (self.mode % 2) == 1:
		# 	self.v = 3.7
		# elif(self.mode % 2) == 0:
		# 	self.v = 8.5

		self.b4_U_rack_yoself()
		if self.obstacles == 1 and self.hrv_zastava == 1:
			self.v = self.v * 0.5
			self.hrv_zastava = 0
			print "PREPREKEEE ", self.obstacles
		if self.obstacles == 1 and self.hrv_zastava == 2:
			self.v = self.v * 1
			self.hrv_zastava = 0
		if self.obstacles == 1 and self.hrv_zastava == 3:
			self.v = self.v * 0.5
			self.hrv_zastava = 0
		if self.obstacles == 1 and self.hrv_zastava == 4:
			self.v = 4.0
			self.hrv_zastava = 0

		

		
		
	


		# ovo je neki moj pokusaj P regulatora na temelju kuta
		#if angle < 5.0:
		#	self.v = 5.0
		#elif angle > 180.0:
		#	self.v = 2.0
		#else:
		#	self.v = (-3.0/175.0)*angle+(178.0/35.0)
		
		# ovaj dio je cista fizika
		# mi = 0.523
		# g = 9.81
		# self.v = math.sqrt(mi*g*R)
		# if self.v > 7:
		# 	self.v = 7


	def calculate_pure_pursuit(self):

		L = 0.3302
		# self.speed2 = self.speed # default: 2.0
		# look-ahead distance ld based on the speed of the vehicle
		
		# ld = self.coefficient * self.v + 0.5 # default: 1.0
		ld = self.v*0.15+0.1
		diff_min = ld
		pp = self.path_position
		self.length2 = len(pp)
		while self.length2 == 0: 
			pp = self.path_position
			self.length2 = len(pp)
		#print "duljina:", self.length2
		self.velocity_controller(pp)
		
		rob_x = self.robot_x
		rob_y = self.robot_y
		rob_qua_x = self.robot_qua_x
		rob_qua_y = self.robot_qua_y
		rob_qua_z = self.robot_qua_z
		rob_qua_w = self.robot_qua_w


		# find position(x,y) on the path that is ld away from the robot.
		for i in range(0, self.length2):
			path_x = pp[i][0]
			path_y = pp[i][1]
			
			path_base_link = self.calculate_pose(path_x, path_y)
			if path_base_link.pose.position.x  < -0.05:
				continue
			
			dist_rp = get_distance(rob_x, rob_y, path_x, path_y)
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

		# calculate alpha1, alpha2 to get steering angle of the robot
		# alpha1 - angle between robot and goal position in radians.
		alpha1 = get_angle_rad(rob_x, self.goal_x, rob_y, self.goal_y)
		quaternion = [rob_qua_x, rob_qua_y, rob_qua_z, rob_qua_w]
		rpy = tf.transformations.euler_from_quaternion(quaternion)
		# alpha2 - robots orientation with respect to map cordinate frame
		alpha2 = rpy[2]
		alpha = alpha1-alpha2
		self.delta = math.atan((2*L*math.sin(alpha))/ld)


		# if self.billy == 1 and self.modul == 6:
		# 	if self.delta >= 0.001:
		# 		self.delta = 0.001
		# 	elif self.delta <= -0.001:
		# 		self.delta = -0.001
		
		# publish speed and steering angle of the robot
		whereto = AckermannDriveStamped()
		whereto.header.stamp = rospy.Time.now()
		whereto.drive.steering_angle = self.delta
		whereto.drive.steering_angle_velocity = 0
		whereto.drive.speed = self.v
		whereto.drive.acceleration = 0
		whereto.drive.jerk = 0
		self.pub.publish(whereto)
		

	def config_callback(self, config, level):
		"""Get configuration parameters from rqt_recongifure"""

		rospy.loginfo("""Reconfiugre Request: {speed}, {coefficient}""".format(**config))
#		self.speed = config.speed
		self.coefficient = config.coefficient
		self.decelerate_dist = config.decelerate_dist
		return config


	def planner_callback(self, data):
		"""
		Get path from path planner node. 
		Make a list of all the poses in the path.
		"""
		
		self.path_position = []
		pom_lista = []
		self.length = len(data.poses)
		for i in range(0, self.length):
			pom_lista.append([data.poses[i].pose.position.x, data.poses[i].pose.position.y])
		x = [p[0] for p in pom_lista]
		y = [p[1] for p in pom_lista]

		f1 = interp1d(x, y)
		xnew = numpy.linspace(x[0],x[-1], num=50, endpoint=True)
		x = xnew
		y = f1(xnew)

   		self.path_position = zip(x,y)
		#pp = self.path_position
		#x = [p[0] for p in pp]
		#y = [p[1] for p in pp]
		#plt.plot(x,y)
		#plt.show()

		#print "pozicija: ", self.path_position
		self.flag = 1


	def odometry_callback(self, data):
		"""Get Robot postition and orientation"""

		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w

	def cmd_callback (self, data):
		self.speed = data.linear.x


	def maptype_callback(self, data):
		self.obstacles = data.data

	def __init__(self):
		"""Create subscribers, publishers and servers."""
		self.circle_of_life = numpy.array([8, 5.6, 3.5, 5.5, 4.0, 6.8, 5.2, 50, 35, 23, 15.7, 8.8, 5.5, 3.8, 3.5, 3.5, 3.6, 4.0, 4.0])
		self.count_dooke = 0
		self.flag = 0
		self.zastavica = 1
		self.mode = 0
		self.cnt = 0
		self.v = 0
		self.turning_points = numpy.array([52.77, -26.19, 41.88, -6.27, 78.99, 36.51, 116.73, 53.93, 136.77, 83.20, 44.29, 46.42, 48.31, 29.69, 35.14, 36.88, 11.34, 17.86, -5.78, 7.45, -4.879, 3.991])
		self.obstacles = 0
		self.modul = 1
		self.hrv_zastava = 0
		self.billy = 0


		srv = Server(RacecarConfig, self.config_callback)
		rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size = 1)
		rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size = 1)
		rospy.Subscriber("/map_type", Int64, self.maptype_callback, queue_size = 1)
		rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.planner_callback, queue_size = 1)
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