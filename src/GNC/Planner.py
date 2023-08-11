#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist , Pose

class Dummy:
    def __init__(self) -> None:
        rospy.init_node('planner')
        self.Xref_publisher = rospy.Publisher('drone/Xref', Pose, queue_size = 1)


    def publish_Xref(self):
        Xref = Pose()
        Xref.position.x = 0
        Xref.position.y = 0
        Xref.position.z = 3
        self.Xref_publisher.publish(Xref)

if __name__=="__main__":
    dummy = Dummy()
    while not rospy.is_shutdown():
        dummy.publish_Xref()