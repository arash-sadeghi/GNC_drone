#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist , Pose
import numpy as np 
import control as ct
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class LQR:
	def __init__(self) -> None:
		rospy.init_node('lqr')
		rospy.Subscriber('drone/gt_pose', Pose , self.pose_callback )
		rospy.Subscriber('drone/Xref', Pose , self.Xref_callback )
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

		self.Xref = None
		self.X = None
		self.K = None
		self.state_dim = 4
		self.input_dim = 4

	def pose_callback(self,data):
		orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.X = np.array([data.position.x, data.position.y, data.position.z , yaw])
	
	def Xref_callback(self,data):
		orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.Xref = np.array([data.position.x, data.position.y, data.position.z , yaw])
		
	def calculate_K(self):
		A = np.zeros((self.state_dim,self.state_dim)) #! this is not true anymore
		B = np.eye(self.input_dim)

		Q = 1.0*np.eye(self.state_dim)
		R =  0.1*np.identity(self.input_dim)

		K, S, E = ct.lqr(A, B, Q,R)
		A_new=(A-B@K)

		self.K = K
		print(f"K {K}")

	def publish_cmd_vel(self):
		if self.Xref is None or self.K is None or self.X is None:
			return

		# dX = self.Xref - self.X
		dX = self.X - self.Xref #! not exatly sure why.

		U = -1*dX@self.K
		# print(f"Xref {self.Xref} X {self.X} dx {dX} U {U}")
		twist = Twist()

		twist.linear.x = U[0] 
		twist.linear.y = U[1] 
		twist.linear.z = U[2] 
		twist.angular.z = U[3]


		self.vel_pub.publish(twist)


if __name__=="__main__":
	lqr = LQR()
	lqr.calculate_K()
	while not rospy.is_shutdown():
		lqr.publish_cmd_vel()
