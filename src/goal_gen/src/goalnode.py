#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from collections import deque
import math




visited_points = list()


def compute_min_distance


class MyNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('goal_publisher', anonymous=True)
        
        # Create subscribers
        self.mapsub = Subscriber('/projected_map', OccupancyGrid)
        self.odomsub = Subscriber('/botcoords', Odometry)

        # Synchronize the subscribers
        # self.ts = TimeSynchronizer([self.mapsub, self.odomsub], 10)
        self.ts = ApproximateTimeSynchronizer([self.mapsub, self.odomsub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        # Create a publisher
        self.pub = rospy.Publisher('/goal', PoseStamped, queue_size=100)

    def callback(self, mapmsg, odommsg):
        global initial
        # if initial is True:
        #     initial_gazebo_position = np.array([int(round(odommsg.pose.pose.position.x)), int(round(odommsg.pose.pose.position.y))])
        #     initial_map_position = np.array([-9,-20])
        #     position_shift = initial_gazebo_position - initial_map_position
        # current_gazebo_position = np.array([int(round(odommsg.pose.pose.position.x)), int(round(odommsg.pose.pose.position.y))])
        # current_map_position = current_gazebo_position - position_shift
        current_map_position = np.array([int(round(odommsg.pose.pose.position.x)), int(round(odommsg.pose.pose.position.y))])

        width = mapmsg.info.width
        height = mapmsg.info.height
        resolution = mapmsg.info.resolution
        origin = mapmsg.info.origin
        
        # Starting random search
        map_data = np.array(mapmsg.data)
        map_data = np.reshape(map_data, (height, width))
        # print(map_data.shape)
        if initial is True:
            newmapdata = np.array(map_data, dtype = np.int64)
            np.savetxt("mapdata.txt", newmapdata)

        print(current_map_position)

        obstacles = np.where(map_data==100)

        # Publishing goal
        # if len(goal)>0:
        print(goal)
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"  # Adjust the frame_id according to your setup
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.orientation.w = 1.0  # Default orientation (no rotation)

        print("publishing")
        self.pub.publish(goal_msg)
        initial = False
        # rospy.loginfo("Received data from topic1: %s and topic2: %s", msg1.data, msg2.data)
        # Get 8x8 grid around bot
        # Iterate through outer layers. Check any black point nearly lies on line connecting
        # Combine the data or process it


if __name__ == '__main__':
    initial = True
    try:
        node = MyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass