#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from math import atan2, sqrt, pi, sin, cos, tan
from f1tenth_gym_ros.msg import RaceInfo

# car parameters
car_width = 0.31
car_length = 0.58
car_length = 0.32

class SpeedVsCurvatureExperiment(object):
    "Drive half circles and visualize paths"

    def __init__(self, driving_angle, robot_velocity, curvature_radius=0):
        # class variables
        self.robot_vel = robot_velocity
        self.velocity_reached = False
        self.paths_published = False

        self.map = None
        self.odom = None
        self.time = -1

        self.robot_path = Path()
        self.cx = 0     # circle center
        self.cy = 0     # circle center
        self.angle = -driving_angle     # turning angle is absolute number
        if curvature_radius == 0:
            self.curv_R = car_length / tan(-self.angle)
        else:
            self.curv_R = curvature_radius
        # self.angle = -atan2(car_length, self.curv_R)  # driving angle (ackermann) - direction
        # self.angle = -0.0321
        print("Driving angle = %f" % self.angle)
        print("Curvature radius = %f m" % self.curv_R)
        
        self.simstart_x = 0
        self.simstart_y = 0
        self.simstart_time = -1
        self.simend_x = 0
        self.simend_y = 0
        self.simend_time = -1

        # ros init
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.race_info_sub = rospy.Subscriber('/race_info', RaceInfo, self.race_info_callback, queue_size=1)

        self.trajectory_pub = rospy.Publisher('/trajectory', Path, queue_size=1)
        self.robot_path_pub = rospy.Publisher('/robot_path', Path, queue_size=1)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        rospy.sleep(0.5)


    def map_callback(self, map):
        self.map = map

    def race_info_callback(self, race_info):
        self.time = race_info.ego_elapsed_time

    def odom_callback(self, odom):
        self.odom = odom
        
        if not self.velocity_reached:
            # robot has only linear.x speed
            if odom.twist.twist.linear.x >= self.robot_vel:
                self.velocity_reached = True
                x = odom.pose.pose.position.x
                y = odom.pose.pose.position.y
                angle = 2*atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
                self.cx, self.cy = self.solve_circle_equation(x, y, angle, self.curv_R)
                self.simstart_x = x
                self.simstart_y = y

                self.simstart_time = self.time
                print("Velocity reached at (%f, %f) in %f s" % (x, y, self.simstart_time))
                print("Circle params: cx = %f cy = %f R = %f" % (self.cx, self.cy, self.curv_R))
                
                curv_time = (2*pi*self.curv_R) / self.robot_vel # half circle
                self.simend_time = self.simstart_time + curv_time
                print("times: %f %f" % (self.simstart_time, self.simend_time))

        # elif abs(odom.twist.twist.linear.x - self.robot_vel) > 0.1 and self.time < self.simend_time + 0.5:
            # print("Warning: Robot is not following speed reference (speed_vs_curvature_systemID.py)")

        if self.velocity_reached and self.time < self.simend_time + 0.5:
            pose_stamped = PoseStamped()
            pose_stamped.pose = odom.pose.pose
            self.robot_path.poses.append(pose_stamped)

    
    def solve_circle_equation(self, x0, y0, theta0, R):
        # 2 solutions - left or right turn
        tg0 = tan(theta0)
        cy = y0 - R / sqrt(1 + tg0**2)
        cx = x0 + (y0-cy)*tg0

        # wrong direction
        if abs( (cx-x0) / (cy-y0) + tg0) > 1e-5:
            print("wrong direction %f %f" %( (cx-x0) / (cy-y0), tg0))
            cy = y0 + R / sqrt(1 + tg0**2)
            cx = x0 + (y0-cy)*tg0
        
        return cx, cy

    def measure_radius(self):
        x0 = self.simstart_x
        y0 = self.simstart_y
        max_dist = 0
        for pose in self.robot_path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            dist = sqrt((x-x0)**2 + (y-y0)**2)
            if dist > max_dist:
                max_dist = dist
        return max_dist / 2
        
    def publish_paths(self):
        self.robot_path.header.frame_id = 'map'
        self.robot_path.header.stamp = rospy.Time.now()
        self.robot_path_pub.publish(self.robot_path)

        trajectory = Path()
        theta0 = atan2(self.simstart_y-self.cy, self.simstart_x-self.cx)
        for i in range(1000):
            pose_stamped = PoseStamped()
            x = self.cx + self.curv_R*cos(theta0 - 2*pi/1000.0*i) # direction
            y = self.cy + self.curv_R*sin(theta0 - 2*pi/1000.0*i) # direction
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            trajectory.poses.append(pose_stamped)
        trajectory.header.frame_id = 'map'
        trajectory.header.stamp = rospy.Time.now()
        self.trajectory_pub.publish(trajectory)


    def run(self):
        if not self.paths_published and self.simend_time != -1 and self.time > self.simend_time:
            self.angle = 0
            self.robot_vel = 0
            self.simend_x = self.odom.pose.pose.position.x
            self.simend_y = self.odom.pose.pose.position.y
            self.publish_paths()
            self.paths_published = True
            print("Measured curvature radius = %f m" % self.measure_radius())
            print("Simulation ended at %f" % self.time)

        acker = AckermannDriveStamped()
        acker.header.stamp = rospy.Time.now()
        if self.velocity_reached:
            acker.drive.speed = self.robot_vel
            acker.drive.steering_angle = self.angle
        else:
            acker.drive.speed = 10*self.robot_vel
            acker.drive.steering_angle = 0
        acker.drive.steering_angle_velocity = 0
        acker.drive.acceleration = 0
        acker.drive.jerk = 0
        self.drive_pub.publish(acker)


if __name__ == '__main__':
    rospy.init_node('speed_vs_curvature_experiment')
    experiment = SpeedVsCurvatureExperiment(0.033, 6.8, 11.4)
    rate = rospy.Rate(150) # 30 Hz
    rate.sleep()
    # real time calculation is dependent on publishing frequency
    while not rospy.is_shutdown():
        experiment.run()
        rate.sleep()
    rospy.spin() # ?