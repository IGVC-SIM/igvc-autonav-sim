#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import ctypes
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import math

def round_accordingly(num):
    if num>0:
        return math.ceil(num)
    elif num<0:
        return math.floor(num)
    else:
        return num

def produce_occupancy_grid(data):
    # Create a list to store the filtered points
    filtered_points = []

    gen = pc2.read_points(data, skip_nans=True)
    int_data = list(gen)
    int_data = np.array([np.array(datai) for datai in int_data])

    print(int_data)

    xmax = np.max(int_data[:,0])
    # print(xmax)
    xmin = np.min(int_data[:,0])
    
    zmax = np.max(int_data[:,2])

    resfactor = 10

    xmax = round_accordingly(xmax*resfactor)
    xmin = round_accordingly(xmin*resfactor)
    zmax = round_accordingly(zmax*resfactor)


    width = xmax+abs(xmin) + 1
    height = zmax + 1
    print(width, height)

    topview = np.zeros((width, height), dtype=np.uint8)
    # print(topview.shape)

    for i, point in enumerate(int_data):
        # xc, yc, zc = point[:3]
        xc = int(math.trunc(point[0]*resfactor))
        if xc<0:
            continue
        yc = int(math.trunc(point[1]*resfactor))
        zc = int(math.trunc(point[2]*resfactor))
        topview[xc,zc]=100
    
    # topview = [list(point) for point in topview]
    topview = topview.flatten().tolist()

    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.frame_id = 'map'
    grid.info.resolution = 1  # Each cell is 1x1 meter
    grid.info.width = width       # Grid width in cells
    grid.info.height = height     # Grid height in cells
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = 0.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0

    grid.data = [0] * (grid.info.width * grid.info.height)  # Initialize all cells to unknown (-1)

    for i, cell in enumerate(topview):
        # print(i)
        grid.data[i] = int(cell)

    # Publish the filtered point cloud
    produce_occupancy_grid_pub.publish(grid)

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_publisher')

    # Subscribe to the input PointCloud2 topic
    point_cloud_sub = rospy.Subscriber('/filtered_point_cloud', PointCloud2, produce_occupancy_grid)

    # Create a publisher for the filtered PointCloud2 topic
    produce_occupancy_grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

    rospy.spin()

