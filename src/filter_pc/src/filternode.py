#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import ctypes

def filter_point_cloud(data):
    # Create a list to store the filtered points
    filtered_points = list()

    gen = pc2.read_points(data, skip_nans=True)
    int_data = list(gen)
    for i, x in enumerate(int_data):
        xc = x[0]
        yc = x[1]
        zc = x[2] 
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        
        if (r > 200 and g > 200 and b > 200) or -1 <= yc <= 1:
            filtered_points.append(x)
    
    
    # Create a new PointCloud2 message for the filtered points
    header = data.header
    fields = data.fields
    new_cloud = pc2.create_cloud(header, fields, filtered_points)
    

    # Publish the filtered point cloud
    filtered_point_cloud_pub.publish(new_cloud)

if __name__ == '__main__':
    rospy.init_node('point_cloud_filter')

    # Subscribe to the input PointCloud2 topic
    point_cloud_sub = rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, filter_point_cloud)

    # Create a publisher for the filtered PointCloud2 topic
    filtered_point_cloud_pub = rospy.Publisher('/filtered_point_cloud', PointCloud2, queue_size=5)

    rospy.spin()

