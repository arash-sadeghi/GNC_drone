#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

class MapExpanderNode:
    def __init__(self):
        rospy.init_node('map_expander')
        
        # Subscribe to the OccupancyGrid topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Publisher for the modified map
        self.map_pub = rospy.Publisher('/expanded_map', OccupancyGrid, queue_size=10)
        
        rospy.spin()
    
    def expand_obstacles(self, original_map):
        # Copy the original map for modification
        expanded_map = OccupancyGrid()
        expanded_map.header = original_map.header
        expanded_map.info = original_map.info
        expanded_map.data = list(original_map.data)  # Convert data to a mutable list
        
        # Iterate through the map data
        for i in range(len(original_map.data)):
            if original_map.data[i] == 100:  # Occupied cell
                x = i % original_map.info.width
                y = i // original_map.info.width
                # Expand obstacle neighbors (adjust the expansion range as needed)
                for dx in range(-5, 5):
                    for dy in range(-5, 5):
                        new_x = x + dx
                        new_y = y + dy
                        if 0 <= new_x < original_map.info.width and 0 <= new_y < original_map.info.height:
                            expanded_map.data[new_y * original_map.info.width + new_x] = 100  # Occupy the neighbor
        
        return expanded_map
    
    def map_callback(self, original_map):
        # Expand obstacles in the map
        expanded_map = self.expand_obstacles(original_map)
        
        # Publish the modified map
        self.map_pub.publish(expanded_map)

if __name__ == '__main__':
    try:
        map_expander = MapExpanderNode()
    except rospy.ROSInterruptException:
        pass
