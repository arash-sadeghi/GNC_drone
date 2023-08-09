#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist , Pose
from std_msgs.msg import Empty 
from sensor_msgs.msg import LaserScan
import numpy as np
import pdb
from tf.transformations import euler_from_quaternion

class Navigator:
	def __init__(self):
		rospy.init_node('drone_navigator')
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.takeoff_pub = rospy.Publisher('drone/takeoff', Empty, queue_size = 1)
		self.land_pub = rospy.Publisher('drone/land', Empty, queue_size = 1)
		rospy.Subscriber('drone/laser', LaserScan , self.LaserScanCallback )
		rospy.Subscriber('drone/gt_pose', Pose , self.PoseCallback )
		self.empty_msg = Empty()
		#! objects colser than self.threshold will be collisio avoided 
		self.threshold = 0.9 
		self.first_data_flag = True
		self.twist = Twist()
		self.LINEAR_VELOCITY=0.5
		self.ANGULAR_VELOCITY=0.5
		self.min_angle=-90
		self.max_angle=90

	def takeoff(self):	
		self.takeoff_pub.publish(self.empty_msg)

	def range2map(self,ranges):
		range_array = np.array(ranges)
		#! mapping input laser ray to laser shape and collision map
		self.ray_map = np.copy( range_array.reshape( self.ray_number , self.ray_number ).T )
		#! row order should be reversed
		self.ray_map = self.ray_map[::-1]
		#! create collision map from self.ray_map
		self.col_map = self.ray_map <= self.threshold

	def PoseCallback(self,data):
		orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.yaw = yaw/np.pi*180 #! we get yaw in degrees
		self.z = data.position.z

	def LaserScanCallback(self,data):
		#! in first incoming laser data, we adjust dimentions of our variables. it wont be processed
		if self.first_data_flag:
			self.first_data_flag = False
			#! assuming that number of vertical rays equal to horizontal rays
			self.ray_number = int(np.sqrt( len(data.ranges) ))
			self.ray_map = np.zeros((self.ray_number , self.ray_number))
			self.col_map = np.zeros((self.ray_number , self.ray_number))
			#! cheat by knowing 5 rays
			# self.rots = np.linspace(self.min_angle , self.max_angle ,self.ray_number)
			self.rots = np.array([-10,-45,-90,45,10])

		else:
			#! this will order ranges in self.ray_map and create self.col_map
			self.range2map(data.ranges)
			# print("[+] coliision map {}".format(self.col_map))
			# print("[+] rospy time {}".format(rospy.get_time()))
	def clear_twist(self):
		#! cleares twist message to avoid confusion with previous values
		self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
		self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0

	def stop(self):
		self.clear_twist()
		self.vel_pub.publish(self.twist)
		print("[+] STOP!")

	def move_forward(self):
		self.clear_twist()
		self.twist.linear.x = 0.5
		self.vel_pub.publish(self.twist)
		print("[+] GO!")
	
	def rotate_yaw(self,angle):
		self.clear_twist()
		self.twist.angular.z = self.ANGULAR_VELOCITY if angle >=0 else -self.ANGULAR_VELOCITY
		initial_yaw = self.yaw
		self.vel_pub.publish(self.twist)

		diff = abs(self.yaw - initial_yaw)  
		diff = diff if diff<=180 else 360-diff
		while abs(diff - abs(angle)) >= 1: #! wait untill we get close to desired angle
			diff = abs(self.yaw - initial_yaw)  
			diff = diff if diff<=180 else 360-diff
			print( diff , abs(angle) )
			pass
		self.stop()
		print("[+] Turned {} degrees".format(angle))
	
	def elevate(self,height):
		self.clear_twist()
		init_z=self.z
		self.twist.linear.z = self.LINEAR_VELOCITY if height >=0 else -self.LINEAR_VELOCITY
		self.vel_pub.publish(self.twist)
		while abs(abs(self.z - init_z) - abs(height)) >= 0.1:
			pass
		self.stop()
		print("[+] Elevated {} from {} to {}".format(height , init_z , self.z))

	def loop(self):
		while not rospy.is_shutdown(): #!
			if not np.any(self.col_map): #! there is no obstacle ahead
				self.move_forward()
			else: 
				self.stop()

				blocks = np.where(self.col_map)
				Z = 2-blocks[0]
				z_action = - Z.mean() / 5 #! /5 is just for avoiding over-recations. it can be changed.
				print("[+] self.col_map {}".format(self.col_map))
				yaw_action = self.rots[blocks[1]].mean()
				print("[+] z_action {} yaw_action {}".format(z_action , yaw_action))
				if abs(z_action) > 0.01:
					self.elevate(z_action)
				elif abs(yaw_action) > 10: #! elif because sometimes executing only one of them solves the problem.
					self.rotate_yaw(yaw_action)
if __name__=="__main__":
	print("[+] navigator started")
	rospy.sleep(10) #! without sleep here, drone wont subscribe to laser
	nav = Navigator()
	rospy.sleep(1) #! without sleep here, drone wont take off
	print("[+] taking off")
	nav.takeoff()
	rospy.sleep(5) #! to avoid taking ground as collision
	print("[+] collision avoidance loop started")
	nav.elevate(1)
	nav.loop()
	rospy.spin()
			
