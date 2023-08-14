#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Twist, Pose
from pdb import set_trace
from direct_colocation import dircol_example_pend
from std_msgs.msg import Empty 
from drone_movements import Basic_Movements
from tf.transformations import euler_from_quaternion,quaternion_from_euler

def dist(p1,p2):
	return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
class Position_handler:
	def __init__(self) -> None:
		rospy.Subscriber('drone/gt_pose', Pose , self.pose_callback )
		self.yaw = 0
		self.position = [0,0]

	def pose_callback(self,data):
		orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		(roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
		self.position = np.array([data.position.x , data.position.y]).squeeze()

	def align_yaw(self):
		e = -1* self.yaw
		return e 

class Optimizer(Basic_Movements):
	def __init__(self):
		rospy.init_node('path_optimizer')
		self.xref_pub = rospy.Publisher('drone/Xref', Pose, queue_size = 1)
		self.takeoff_pub = rospy.Publisher('drone/takeoff', Empty, queue_size = 1)
		super().__init__(None , self.takeoff_pub , None)
		rospy.sleep(1) #! without sleep here, drone wont take off
		print("[+] taking off")
		self.takeoff()
		rospy.sleep(5) #! to avoid taking ground as collision
		path_sub = rospy.Subscriber('/path', PoseArray, self.path_callback)
		self.yaw_aligner = Position_handler()
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

		for i in range(len(X_ref[0,:])):

			x_ref = np.array([X_ref[0,i] , X_ref[1,i]])
			print(f"[optmizer] publishing xref {x_ref}")
			while dist(x_ref , self.yaw_aligner.position) > 0.1:
				tmp = Pose()
				tmp.position.x = x_ref[0]
				tmp.position.y = x_ref[1]
				tmp.position.z = 1			
				self.xref_pub.publish(tmp)


		print("reached destination")
		self.finished = True

if __name__ == '__main__':
	print("[++] path optimizer started")
	opt = Optimizer()
	# init_path = np.load("/home/arash/catkin_ws/src/GNC_drone_navigation/data/example_path.npy")
	# x_sol, u_sol, X_ref, U_ref, T_ref= dircol_example_pend(init_path*0.05)
	# dirtran_example(init_path)
	rospy.spin()
