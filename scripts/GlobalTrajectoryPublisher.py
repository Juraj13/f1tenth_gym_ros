#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import sqrt, atan2, sin, atan

car_length = 0.32

class GlobalTrajectoryPublisher(object):
    def __init__(self):
        self.odom = None
        self.speed = -1
        self.last_index = -1
        self.trajectory = np.empty((0, 3))
        self.v = [1.2, 1.8, 2.2, 2.4, 2.8, 3.3, 4.2, 6.0, 7.0, 10.0]
        self.R = [1.0, 1.55, 1.77, 2.5, 3.36, 8.8, 11.4, 20.0, 30.0]

        self.got_info = False
        self.trajectory = []
        with open('/home/ivan/catkin_ws/src/f1tenth_gym_ros/trajectory.csv', 'r') as file:
            for line in file:
                xyr_str = line.replace('\n', '').split(', ')
                x = float(xyr_str[0])
                y = float(xyr_str[1])
                r = float(xyr_str[2])
                self.trajectory.append([x,y,r])
        print("length of trajectory %d" % len(self.trajectory))

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.trajectory_pub = rospy.Publisher('/global_traj', Path, queue_size=1)
        self.map_info_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)

        rospy.sleep(0.5)

    def publish_trajectory(self):
        print("path published")
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        self.trajectory = self.fit2map(self.trajectory)  #bilo je trajectory (bez self)
        for point in self.trajectory:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path.poses.append(pose)
        self.trajectory_pub.publish(path)

    def map_callback(self, map):
        print('got map info')
        map_info = map.info
        self.resolution = map_info.resolution
        self.origin = map_info.origin.position
        self.got_info = True

    def odom_callback(self, odom):
        self.odom = odom.pose.pose.position
        self.orientation = 2*atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)


    def fit2map(self, trajectory):
        if not self.got_info:
            print("Didnt get map info")
            return -1

        for point in trajectory:
            point[0] = self.origin.x + point[0]*self.resolution
            point[1] = self.origin.y + point[1]*self.resolution
            point[2] = point[2] * self.resolution
        return trajectory


    def calc_traj_pos(self):
        min_dist = 1e9
        index = -1
        if self.last_index == -1:
            for i in range(0, len(self.trajectory)):
                dist = sqrt((self.trajectory[i][0]-self.odom.x)**2 + (self.trajectory[i][1]-self.odom.y)**2)
                if dist < min_dist:
                    min_dist = dist
                    index = i
        else:
            for i in range(self.last_index, self.last_index+30):
                if i  >= len(self.trajectory):
                    ind = i - len(self.trajectory)
                else:
                    ind = i
                dist = sqrt((self.trajectory[ind][0]-self.odom.x)**2 + (self.trajectory[ind][1]-self.odom.y)**2)
                if dist < min_dist:
                    min_dist = dist
                    index = ind

        return index, min_dist

    def calc_speed(self, R, dist, index):
        for i in range(len(self.R)):
            if abs(R) < self.R[i]:
                v = self.v[i]
        if abs(R) > self.R[len(self.R)-1]:
            v = self.v[len(self.v)-1]
        print(v, R)
        look_ahead = index + int(v)*15
        if look_ahead >= len(self.trajectory):
            look_ahead -= len(self.trajectory)
        R_ahead = self.trajectory[look_ahead][2]
        if R_ahead < R:
            for i in range(len(self.R)):
                if abs(R_ahead) < self.R[i]:
                    v = self.v[i]
            if abs(R) > self.R[-1]:
                v = self.v[len(self.v)-1]
        
        return v


    def calc_side(self, index):
        if index == len(self.trajectory)-1:
            vx1 = self.trajectory[0][0] - self.trajectory[index][0]
            vy1 = self.trajectory[0][1] - self.trajectory[index][1]
        else:
            vx1 = self.trajectory[index+1][0] - self.trajectory[index][0]
            vy1 = self.trajectory[index+1][1] - self.trajectory[index][1]
        vx2 = self.odom.x - self.trajectory[index][0]
        vy2 = self.odom.y - self.trajectory[index][1]

        if vx1*vy2 - vy1*vx2 > 0:
            return 1
        else:
            return -1


    # pure pursuit
    def calc_angle(self, i, goal_i):
        if goal_i >= len(self.trajectory):
            goal_i -= len(self.trajectory)
        goal_x = self.trajectory[goal_i][0]
        goal_y = self.trajectory[goal_i][1]
        ld = sqrt( (goal_x-self.trajectory[i][0])**2 + (goal_y-self.trajectory[i][1])**2 )
        alpha1 = atan2(goal_y - self.odom.y, goal_x - self.odom.x)
        alpha2 = self.orientation
        alpha = alpha1-alpha2
        angle = atan((2*car_length*sin(alpha))/ld)
        return angle
        
    
    def run(self):
        i, dist = self.calc_traj_pos()
        self.last_index = i
        self.speed = self.calc_speed(self.trajectory[i][2], dist, i)
        side = self.calc_side(i)
        self.angle = self.calc_angle(i, i+7)
        
        ack = AckermannDriveStamped()
        ack.header.stamp = rospy.Time.now()
        ack.drive.speed = 1.3 #self.speed
        ack.drive.steering_angle = self.angle
        self.drive_pub.publish(ack)


if __name__ == '__main__':
    rospy.init_node('global_trajectory_publisher')
    trajectory_publisher = GlobalTrajectoryPublisher()
    rate = rospy.Rate(10) # 1 Hz
    rate.sleep()
    while not rospy.is_shutdown():
        connections = trajectory_publisher.trajectory_pub.get_num_connections()
        if connections > 0 and trajectory_publisher.got_info:
            trajectory_publisher.publish_trajectory()
            break
        rate.sleep()
    while not rospy.is_shutdown():
        trajectory_publisher.run()
    rospy.spin() # ?