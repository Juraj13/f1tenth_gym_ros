#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import pyplot
from math import atan2, pi
import rospy
import numpy
import tf
import message_filters as mf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class VirtualWall(object):

	def callback(self, odom_data, map_data):
		"""Get Robot postition and orientation and read map data"""
		# get robot pose
		self.robot_x = odom_data.pose.pose.position.x
		self.robot_y = odom_data.pose.pose.position.y
		self.robot_qua_x = odom_data.pose.pose.orientation.x
		self.robot_qua_y = odom_data.pose.pose.orientation.y
		self.robot_qua_z = odom_data.pose.pose.orientation.z
		self.robot_qua_w = odom_data.pose.pose.orientation.w
		
		#get map info
		self.map_width = map_data.info.width
	        self.map_height = map_data.info.height
	        self.map_resolution = map_data.info.resolution
	        self.map_origin = map_data.info.origin.position
	        self.load_time = map_data.info.map_load_time
		self.mapa = map_data.data
		self.mapa = numpy.array(self.mapa).reshape(self.map_width, self.map_height)
		
		# calculate robots position in pixels
		self.ind_x = int((self.robot_x - self.map_origin.x) / self.map_resolution)
		self.ind_y = int((self.robot_y - self.map_origin.y) / self.map_resolution)
		
		# publish map info
		nova_mapa = OccupancyGrid()
		nova_mapa.header.stamp = rospy.Time.now()
		nova_mapa.header.frame_id = "map_v2"
		nova_mapa.info.map_load_time = self.load_time
		nova_mapa.info.width = self.map_width
		nova_mapa.info.height = self.map_height
		nova_mapa.info.resolution = self.map_resolution
		nova_mapa.info.origin.position = self.map_origin
		
		edge1, edge2 = self.find_nearest_track_edges([self.ind_x, self.ind_y])
		self.mapa = self.build_wall(edge1, edge2)
		nova_mapa.data = numpy.array(self.mapa).reshape(self.map_width*self.map_height)
		self.pub.publish(nova_mapa)
		
		# publish goal point
		goal = PoseStamped()
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = "map"
		goal.pose.position.x = self.robot_x-0.5
		goal.pose.position.y = self.robot_y
		goal.pose.position.z = 0.0
		goal.pose.orientation.x = self.robot_qua_x
		goal.pose.orientation.y = self.robot_qua_y
		goal.pose.orientation.z = self.robot_qua_z
		goal.pose.orientation.w = self.robot_qua_w
		
		self.pub2.publish(goal)
		self.cont = 0
		print "here"

	def find_nearest_track_edges(self, start):
	    visited = np.zeros([self.map_width, self.map_height])
	    edge1, visited = self.find_nearest_track_edge(start, visited)
	    edge2, visited = self.find_nearest_track_edge(start, visited)
	    return edge1, edge2

	def find_nearest_track_edge(self, start, visited):
	    queue = []
	    visited[start[0]][start[1]] = 1
	    queue.append(start)
	    
	    while queue:
		s = queue.pop(0)

		if self.mapa[s[0]][s[1]] == 100: # occupied
		    visited = self.mark_edge_as_visited(s)
		    return s, visited
		
		if visited[ s[0]+1 ][ s[1] ] == 100:
		    queue.append([s[0]+1, s[1]])
		    visited[s[0]+1][s[1]] = 1
		if visited[ s[0] ][ s[1]+1 ] == 100:
		    queue.append([s[0], s[1]+1])
		    visited[s[0]+1][s[1]+1] = 1
		if visited[ s[0]-1 ][ s[1] ] == 100:
		    queue.append([s[0]-1, s[1]])
		    visited[s[0]-1][s[1]] = 1
		if visited[ s[0] ][ s[1]-1 ] == 100:
		    queue.append([s[0], s[1]-1])
		    visited[s[0]][s[1]-1] = 1

	    return None, visited

	def mark_edge_as_visited(self, start):
	    visited = np.zeros([self.map_width, self.map_height])
	    queue = [start]
	    visited[start[0]][start[1]] = 1

	    while queue:
		s = queue.pop(0)

		if self.mapa[s[0]+1][s[1]] == 0 or self.mapa[s[0]][s[1]+1] == 0 or \
		   self.mapa[s[0]-1][s[1]] == 0 or self.mapa[s[0]][s[1]-1] == 0:  #free
		   visited[s[0]][s[1]] = 1
		else:
		    continue

		if self.mapa[s[0]+1][s[1]] == 100 and visited[s[0]+1][s[1]] == 100: # occupied
		    queue.append([s[0]+1, s[1]])
		    visited[s[0]+1][s[1]] = 1
		if self.mapa[s[0]][s[1]+1] == 100 and visited[s[0]][s[1]+1] == 100: # occupied
		    queue.append([s[0], s[1]+1])
		    visited[s[0]][s[1]+1] = 1
		if self.mapa[s[0]-1][s[1]] == 100 and visited[s[0]-1][s[1]] == 100: # occupied
		    queue.append([s[0]-1, s[1]])
		    visited[s[0]-1][s[1]] = 1
		if self.mapa[s[0]][s[1]-1] == 100 and visited[s[0]][s[1]-1] == 100: # occupied
		    queue.append([s[0], s[1]-1])
		    visited[s[0]][s[1]-1] = 1

		if self.mapa[s[0]+1][s[1]+1] == 100 and visited[s[0]+1][s[1]+1] == 100: # occupied
		    queue.append([s[0]+1, s[1]+1])
		    visited[s[0]+1][s[1]+1] = 1
		if self.mapa[s[0]+1][s[1]-1] == 100 and visited[s[0]+1][s[1]-1] == 100: # occupied
		    queue.append([s[0]+1, s[1]-1])
		    visited[s[0]+1][s[1]-1] = 1
		if self.mapa[s[0]-1][s[1]+1] == 100 and visited[s[0]-1][s[1]+1] == 100: # occupied
		    queue.append([s[0]-1, s[1]+1])
		    visited[s[0]-1][s[1]+1] = 1
		if self.mapa[s[0]-1][s[1]-1] == 100 and visited[s[0]-1][s[1]-1] == 100: # occupied
		    queue.append([s[0]-1, s[1]-1])
		    visited[s[0]-1][s[1]-1] = 1

	    return visited

	def build_wall(self, edge1, edge2):
	    if edge1 == edge2:
		return self.mapa

	    self.mapa[edge1[0]][edge1[1]] = 100 # occupied
	    direction = atan2(edge2[1] - edge1[1], edge2[0] - edge1[0])
	    direction = direction * 180/pi
	    if direction >= -45 and direction < 45:
		return self.build_wall([edge1[0]+1, edge1[1]], edge2)
	    if direction >= 45 and direction < 115:
		return self.build_wall([edge1[0], edge1[1]+1], edge2)
	    if direction >= 115 or direction < -115:
		return self.build_wall([edge1[0]-1, edge1[1]], edge2)
	    if direction >= -115 and direction < -45:
		return self.build_wall([edge1[0], edge1[1]-1], edge2)

        def __init__(self):
        	
        	# subscribers and publishers
        	subs = [mf.Subscriber('/odom', Odometry), mf.Subscriber('/map', OccupancyGrid)]
        	self.ts = mf.ApproximateTimeSynchronizer(subs, 1, 1)
        	self.ts.registerCallback(self.callback)
        	
        	self.pub = rospy.Publisher('/map_v2', OccupancyGrid, queue_size=1)
        	self.pub2 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        	self.cont = 1
        	
        	while self.cont == 1:
        		continue
        	
        	
if __name__ == "__main__":

	rospy.init_node("virtual_wall")
	vw = VirtualWall()
