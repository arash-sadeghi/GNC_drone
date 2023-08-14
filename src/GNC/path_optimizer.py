#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from pdb import set_trace
from direct_colocation import dircol_example_pend


def path_callback(msg):
    init_path = []
    if len(msg.poses) == 0:
        print("[--] recieved path is empty")
        return
    for node in msg.poses:
        init_path.append([ [node.position.x] , [node.position.y] ])
    init_path = np.array(init_path).squeeze() 
    # set_trace()
    x_sol, u_sol, X_ref, U_ref, T_ref= dircol_example_pend(init_path)    
    print(f"optimized path {X_ref}")

if __name__ == '__main__':
    rospy.init_node('path_optimizer')
    path_sub = rospy.Subscriber('/path', PoseArray, path_callback)
    print("[++] path optimizer started")

    # init_path = np.load("/home/arash/catkin_ws/src/GNC_drone_navigation/data/example_path.npy")
    # x_sol, u_sol, X_ref, U_ref, T_ref= dircol_example_pend(init_path*0.05)
    # dirtran_example(init_path)
    rospy.spin()
