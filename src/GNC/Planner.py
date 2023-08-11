#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist , Pose
from tf.transformations import quaternion_from_euler

class Dummy:
    def __init__(self) -> None:
        rospy.init_node('planner')
        self.Xref_publisher = rospy.Publisher('drone/Xref', Pose, queue_size = 1)


    def publish_Xref(self):
        Xref = Pose()
        Xref.position.x = 0
        Xref.position.y = 0
        Xref.position.z = 3
        yaw_ref = 0
        quaternion_ref = quaternion_from_euler(0,0,yaw_ref)
        Xref.orientation.x = quaternion_ref[0]
        Xref.orientation.y = quaternion_ref[1]
        Xref.orientation.z = quaternion_ref[2]
        Xref.orientation.w = quaternion_ref[3]
        self.Xref_publisher.publish(Xref)

if __name__=="__main__":
    dummy = Dummy()
    while not rospy.is_shutdown():
        dummy.publish_Xref()