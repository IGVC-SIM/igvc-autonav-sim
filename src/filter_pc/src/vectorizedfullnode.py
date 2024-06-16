#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import ctypes

def get_rgb_vectorized(packedcolours):
    # Ensure the array is C-contiguous
    packedcolours = np.ascontiguousarray(packedcolours)
    # Vectorized conversion of float32 to uint32
    packed_ints = packedcolours.view(np.uint32)
    # Bitwise operations
    r = (packed_ints & 0x00FF0000) >> 16
    g = (packed_ints & 0x0000FF00) >> 8
    b = packed_ints & 0x000000FF
    return np.stack((r, g, b), axis=-1)

def checkwhite_vectorized(rgb):
    return np.all(rgb >= 180, axis=-1)

def manipulate_vectorized(x, z_threshold, y_threshold):
    # Ensure the input array is C-contiguous
    x = np.ascontiguousarray(x)

    # Extract relevant columns
    z_vals = x[:, 2]
    y_vals = x[:, 1]
    packed_colours = x[:, 3]

    # Conditions
    z_condition = z_vals < z_threshold

    rgb_values = get_rgb_vectorized(packed_colours)
    white_condition = checkwhite_vectorized(rgb_values)
    y_condition = y_vals <= y_threshold

    # Ensure all conditions have the same shape
    z_condition = np.asarray(z_condition, dtype=bool)
    white_condition = np.asarray(white_condition, dtype=bool)
    y_condition = np.asarray(y_condition, dtype=bool)

    # Combine conditions
    combined_condition = z_condition & (~white_condition | ~y_condition)

    # Apply manipulation
    newx = x[:, :3].copy()
    newx[combined_condition, 1] -= 200

    # Filter out elements where z_condition is not met
    filtered_newx = newx[z_condition]

    return filtered_newx



def filter_point_cloud(data):
    # Create a list to store the filtered points
    filtered_points = list()

    gen = pc2.read_points(data, skip_nans=True)
    int_data = list(gen)
    int_data = np.array(int_data)
    y_values = int_data[:,1]
    y_threshold = np.max(y_values) * 0.6
    y_threshold2 = np.min(y_values) * 0.8

    z_values = int_data[:,2]
    z_threshold = np.max(z_values) * 0.9

    filtered_points = manipulate_vectorized(int_data, z_threshold, y_threshold)  
    
    # Create a new PointCloud2 message for the filtered points
    header = data.header
    fields = data.fields
    new_cloud = pc2.create_cloud(header, fields, filtered_points)
    

    # Publish the filtered point cloud
    filtered_point_cloud_pub.publish(new_cloud)

if __name__ == '__main__':
    rospy.init_node('point_cloud_rotated')

    # Subscribe to the input PointCloud2 topic
    point_cloud_sub = rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, filter_point_cloud)

    # Create a publisher for the filtered PointCloud2 topic
    filtered_point_cloud_pub = rospy.Publisher('/altered_point_cloud', PointCloud2, queue_size=10)

    rospy.spin()

