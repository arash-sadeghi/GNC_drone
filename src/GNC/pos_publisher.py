#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

if __name__ == '__main__':

    rospy.init_node('pos_publisher')


    g_pub = rospy.Publisher('/goal_position', PoseStamped, queue_size=1)

    g = PoseStamped()
    2.7 
    7.6
    
    
    g.pose.position.x = 15.4 + (-3.6)#2.7
    g.pose.position.y = 13.8 + (-3)#7.6

    x = Int32MultiArray()


    print(f"will go to {g}")
    while not rospy.is_shutdown():
        g_pub.publish(g)
