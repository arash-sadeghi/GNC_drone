from geometry_msgs.msg import Twist 
from CONST import CONST_class

class Basic_Movements: #* publishes twist message for called basic movement
	def __init__(self,vel_pub,takeoff_pub,land_pub):
		self.twist = Twist()
		self.vel_pub = vel_pub
		self.takeoff_pub = takeoff_pub

	def clear_twist_msg(self):
		self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
		self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0

	def stop(self):
		self.clear_twist_msg()
		self.vel_pub.publish(self.twist)

	def move_forward(self):
		self.clear_twist_msg()
		self.twist.linear.x = CONST_class.MAX_X_VEL
		self.vel_pub.publish(self.twist)

	def takeoff(self):	
		self.takeoff_pub.publish(self.empty_msg)