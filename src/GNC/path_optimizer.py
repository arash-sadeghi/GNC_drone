#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Twist, Pose
from pdb import set_trace
from direct_colocation import dircol_example_pend
from std_msgs.msg import Empty 
from drone_movements import Basic_Movements
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class Yaw_aligner:
	def __init__(self) -> None:
		rospy.Subscriber('drone/gt_pose', Pose , self.pose_callback )
		self.yaw = 0

	def pose_callback(self,data):
		orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		(roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

	def align_yaw(self):
		e = -1* self.yaw
		return e 

class Optimizer(Basic_Movements):
	def __init__(self):
		rospy.init_node('path_optimizer')
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.takeoff_pub = rospy.Publisher('drone/takeoff', Empty, queue_size = 1)
		self.land_pub = rospy.Publisher('drone/land', Empty, queue_size = 1)
		super().__init__(self.vel_pub , self.takeoff_pub , self.land_pub)
		rospy.sleep(1) #! without sleep here, drone wont take off
		print("[+] taking off")
		self.takeoff()
		rospy.sleep(5) #! to avoid taking ground as collision
		path_sub = rospy.Subscriber('/path', PoseArray, self.path_callback)
		self.yaw_aligner = Yaw_aligner()
		self.finished = False

	def path_callback(self,msg):
		if self.finished:
			return
		
		init_path = []
		if len(msg.poses) == 0:
			print("[--] recieved path is empty")
			return
		for node in msg.poses:
			init_path.append([ [node.position.x] , [node.position.y] ])
		init_path = np.array(init_path).squeeze() 
		# set_trace()
		x_sol, u_sol, X_ref, U_ref, T_ref= dircol_example_pend(init_path)    
		start_time = rospy.Time.now().to_sec()
		t = 0
		
		# print(f"time dur {rospy.Time.now().to_sec()} ",type(rospy.Time.now().to_sec()) , T_ref[-1] , type(T_ref[-1]))
		while T_ref[-1] - t >= 0:
			t = rospy.Time.now().to_sec() - start_time
			vel = Twist()
			control_input = u_sol.vector_values([t])
			vel.linear.x =  control_input[0][0]
			vel.linear.y =  control_input[1][0]
			vel.angular.z = self.yaw_aligner.align_yaw()
			self.vel_pub.publish(vel)
			print(f"time dur {t} u {u_sol.vector_values([t])} published {vel}")
		self.stop()
		print("reached destination")
		self.finished = True
if __name__ == '__main__':
	print("[++] path optimizer started")
	opt = Optimizer()
	# init_path = np.load("/home/arash/catkin_ws/src/GNC_drone_navigation/data/example_path.npy")
	# x_sol, u_sol, X_ref, U_ref, T_ref= dircol_example_pend(init_path*0.05)
	# dirtran_example(init_path)
	rospy.spin()
