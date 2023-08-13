#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int32MultiArray

path = None
def map_callback(msg):
    global path
    print("map received")
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    map_array = np.array(msg.data)
    
    map_array[300 * width + 300] = 200

    while path is None:
        print("path not availble")
        # return
    print("path",path)
    for _ in path:
        map_array[_] = 200
    
    map_data = map_array.reshape((height, width))
    print(">",np.where(map_data==200))
    plt.imshow(map_data, cmap='gray', origin='lower', extent=(0, width*resolution, 0, height*resolution))
    plt.colorbar()
    plt.title('Occupancy Grid Map')
    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.show()
    plt.savefig('/home/arash/catkin_ws/src/my_ros_astar/data/saved_fig.png')

def path_callback(msg):
    global path
    path = msg.data
    print("path received")

def main():
    rospy.init_node('map_viz')
    
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/path", Int32MultiArray, path_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
