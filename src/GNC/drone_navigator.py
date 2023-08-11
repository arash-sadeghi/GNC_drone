#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist , Pose
from std_msgs.msg import Empty 
from sensor_msgs.msg import LaserScan
import numpy as np
import pdb
from tf.transformations import euler_from_quaternion
from drone_movements import Basic_Movements
from CONST import CONST_class

class Navigator(Basic_Movements):
	def __init__(self):
		rospy.init_node('drone_navigator')
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.takeoff_pub = rospy.Publisher('drone/takeoff', Empty, queue_size = 1)
		self.land_pub = rospy.Publisher('drone/land', Empty, queue_size = 1)
		super().__init__(self.vel_pub , self.takeoff_pub , self.land_pub)
		rospy.Subscriber('drone/laser', LaserScan , self.LaserScanCallback )
		rospy.Subscriber('drone/gt_pose', Pose , self.PoseCallback )
		self.empty_msg = Empty()
		self.twist = Twist()
		self.position = Pose()

	def PoseCallback(self,data):
		# orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		# (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		# self.yaw = yaw/np.pi*180 #! we get yaw in degrees
		# self.position = data.position
		pass

	def LaserScanCallback(self,data):
		pass

	def rotate_yaw(self,angle):
		self.clear_twist_msg()
		self.twist.angular.z = CONST_class.ANGULAR_VELOCITY if angle >=0 else -CONST_class.ANGULAR_VELOCITY
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
		self.clear_twist_msg()
		init_z=self.position.z
		self.twist.linear.z = CONST_class.LINEAR_VELOCITY if height >=0 else -CONST_class.LINEAR_VELOCITY
		self.vel_pub.publish(self.twist)
		while abs(abs(self.position.z - init_z) - abs(height)) >= 0.1:
			pass
		self.stop()
		print("[+] Elevated {} from {} to {}".format(height , init_z , self.position.z))

	def loop(self):
		# self.move_forward()
		pass

if __name__=="__main__":
	print("[+] navigator started")
	rospy.sleep(10) #! without sleep here, drone wont subscribe to laser
	nav = Navigator()
	rospy.sleep(1) #! without sleep here, drone wont take off
	print("[+] taking off")
	nav.takeoff()
	rospy.sleep(5) #! to avoid taking ground as collision
	print("[+] collision avoidance loop started")
	# nav.elevate(1)
	# nav.loop()
	rospy.spin()
			
