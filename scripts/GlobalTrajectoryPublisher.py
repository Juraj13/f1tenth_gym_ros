#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class GlobalTrajectoryPublisher(object):
    def __init__(self):
        self.trajectory_pub = rospy.Publisher('/global_traj', Path, queue_size=1)
        self.map_info_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.got_info = False
        self.trajectory = []
        with open('/home/ivan/catkin_ws/src/f1tenth_gym_ros/trajectory.csv', 'r') as file:
            for line in file:
                xy_str = line.replace('\n', '').split(', ')
                x = float(xy_str[0])
                y = float(xy_str[1])
                self.trajectory.append([x,y])


    def publish_trajectory(self):
        print("path published")
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        trajectory = self.fit2map(self.trajectory)
        for point in trajectory:
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


    def fit2map(self, trajectory):
        if not self.got_info:
            print("Didnt get map info")
            return -1

        for point in trajectory:
            point[0] = self.origin.x + point[0]*self.resolution
            point[1] = self.origin.y + point[1]*self.resolution
        return trajectory

if __name__ == '__main__':
    rospy.init_node('global_trajectory_publisher')
    trajectory_publisher = GlobalTrajectoryPublisher()
    rate = rospy.Rate(1) # 1 Hz
    rate.sleep()
    while not rospy.is_shutdown():
        connections = trajectory_publisher.trajectory_pub.get_num_connections()
        if connections > 0 and trajectory_publisher.got_info:
            trajectory_publisher.publish_trajectory()
            break
        rate.sleep()
    rospy.spin() # ?