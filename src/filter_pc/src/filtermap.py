#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class FilteredMapNode:
    def __init__(self):
        rospy.init_node('filtered_map_node', anonymous=True)
        
        self.filtered_map = None
        self.filtered_array = None
        
        self.map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.map_callback)
        self.map_pub = rospy.Publisher('/filtered_map', OccupancyGrid, queue_size=1)
        
    def map_callback(self, msg):
        if self.filtered_map is None:
            self.filtered_map = msg
            self.filtered_array = np.full((msg.info.height, msg.info.width), -1, dtype=np.int8)
        
        # Convert incoming data to 2D numpy array
        incoming_array = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # Calculate offset between incoming map and filtered map
        x_offset = int((msg.info.origin.position.x - self.filtered_map.info.origin.position.x) / msg.info.resolution)
        y_offset = int((msg.info.origin.position.y - self.filtered_map.info.origin.position.y) / msg.info.resolution)
        
        # Determine the new size of the filtered map
        new_width = max(self.filtered_array.shape[1], x_offset + msg.info.width)
        new_height = max(self.filtered_array.shape[0], y_offset + msg.info.height)
        
        # Create a new array with the larger size
        new_filtered_array = np.full((new_height, new_width), -1, dtype=np.int8)
        
        # Copy existing data to the new array
        existing_height, existing_width = self.filtered_array.shape
        new_filtered_array[
            max(0, -y_offset):max(0, -y_offset) + existing_height,
            max(0, -x_offset):max(0, -x_offset) + existing_width
        ] = self.filtered_array[:min(existing_height, new_height - max(0, -y_offset)),
                                :min(existing_width, new_width - max(0, -x_offset))]
        
        # Update with new known cells
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                new_y = y_offset + i
                new_x = x_offset + j
                if 0 <= new_y < new_height and 0 <= new_x < new_width:
                    if incoming_array[i, j] != -1 and new_filtered_array[new_y, new_x] == -1:
                        new_filtered_array[new_y, new_x] = incoming_array[i, j]
        
        # Update filtered map metadata
        self.filtered_map.info.width = new_width
        self.filtered_map.info.height = new_height
        self.filtered_map.info.origin.position.x = min(self.filtered_map.info.origin.position.x, msg.info.origin.position.x)
        self.filtered_map.info.origin.position.y = min(self.filtered_map.info.origin.position.y, msg.info.origin.position.y)
        
        # Flatten the array and update the filtered map data
        self.filtered_map.data = new_filtered_array.flatten().tolist()
        self.filtered_array = new_filtered_array
        
        # Publish the updated filtered map
        self.map_pub.publish(self.filtered_map)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = FilteredMapNode()
        node.run()
    except rospy.ROSInterruptException:
        pass