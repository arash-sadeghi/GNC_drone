#!/usr/bin/env python3
#!/home/arash/catkin_ws/src/GNC_drone_navigation/env/bin/ python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped , Pose, PoseArray
from std_msgs.msg import Int32MultiArray
import heapq
import numpy as np
from pdb import set_trace

class AStarPathPlanner:
	def __init__(self):
		rospy.init_node('astar')

		self.map = None
		self.start = None
		self.goal = None

		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
		self.start_sub = rospy.Subscriber('drone/gt_pose', Pose , self.start_callback )      
		self.goal_sub = rospy.Subscriber('/goal_position', PoseStamped, self.goal_callback)
		self.path_pub = rospy.Publisher('/path', PoseArray, queue_size=10)

	def map_callback(self, msg):
		self.map = msg

	def start_callback(self, msg):
		if self.map is None:
			return 
		x = msg.position.x- self.map.info.origin.position.x
		y = msg.position.y- self.map.info.origin.position.y
		self.start = (int(x/self.map.info.resolution) , int(y/self.map.info.resolution))

	def goal_callback(self, msg):
		if self.map is None:
			return 
		self.goal = (int(msg.pose.position.x/self.map.info.resolution), int(msg.pose.position.y/self.map.info.resolution))

	def heuristic(self, node):
		return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])

	def astar(self):
		if self.map is None or self.start is None or self.goal is None:
			return []
		open_heap = [(0, self.start)]
		came_from = {}
		g_scores = {node: float('inf') for node in self.get_nodes()}
		g_scores[self.start] = 0
		f_scores = {node: float('inf') for node in self.get_nodes()}
		f_scores[self.start] = self.heuristic(self.start)

		while open_heap:
			current_f, current = heapq.heappop(open_heap)

			if current == self.goal:
				path = []
				# print(f"came_from {came_from}")
				while current in came_from:
					path.append(current)
					current = came_from[current]
				path.append(self.start)
				# print(f"path {path}")
				path.reverse()
				return path

			for neighbor in self.get_neighbors(current):
				tentative_g = g_scores[current] + self.heuristic(neighbor)
				if tentative_g < g_scores[neighbor]:
					came_from[neighbor] = current
					g_scores[neighbor] = tentative_g
					f_scores[neighbor] = g_scores[neighbor] + self.heuristic(neighbor)
					if neighbor not in [node for _, node in open_heap]:
						heapq.heappush(open_heap, (f_scores[neighbor], neighbor))

		print(f"path not found from {self.start} to {self.goal}")
		return []

	def get_nodes(self):
		nodes = []
		for x in range(self.map.info.width):
			for y in range(self.map.info.height):
				if self.map.data[x + y * self.map.info.width] == 0:
					nodes.append((x, y))
		return nodes

	def get_neighbors(self, node):
		neighbors = []
		x, y = node
		for dx in [-1, 0, 1]:
			for dy in [-1, 0, 1]:
				new_x = x + dx
				new_y = y + dy
				if (new_x, new_y) != node and (0 <= new_x < self.map.info.width) and (0 <= new_y < self.map.info.height) and self.map.data[new_x + new_y * self.map.info.width] == 0:
					neighbors.append((new_x, new_y))
		return neighbors

	def publish_path(self, path):
		# path_msg = Int32MultiArray(data=[node[0] + node[1] * self.map.info.width for node in path])
		# path_msg = Int32MultiArray(data=[node for node in path])
		path_msg = PoseArray()
		for node in path:
			tmp = Pose()
			tmp.position.x = node[0]*self.map.info.resolution + self.map.info.origin.position.x
			tmp.position.y = node[1]*self.map.info.resolution + self.map.info.origin.position.y
			path_msg.poses.append(tmp)
		# print(f"path calculated and published succesfully {path_msg}")	
		# print(f"path calculated and published succesfully")	

		self.path_pub.publish(path_msg)

	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			path = self.astar()
			# np.save("/home/arash/catkin_ws/src/GNC_drone_navigation/data/example_path.npy" , path) #! continuesly saving file
			rate.sleep()
			if len(path) > 0:
				self.publish_path(path)
				# break #* this makes sure the astar will be only run once

if __name__ == '__main__':
	try:
		print("[+] astar started")
		planner = AStarPathPlanner()
		planner.run()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()