#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import ctypes

def get_rgb(packedcolour):
    # cast float32 to int so that bitwise operations are possible    
    s = struct.pack('>f' ,packedcolour)
    i = struct.unpack('>l',s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000)>> 16
    g = (pack & 0x0000FF00)>> 8
    b = (pack & 0x000000FF)
    return r, g, b

def checkwhite(rgb):
    for i in rgb:
        if i<180:
            return False
    return True

def filter_point_cloud(data):
    # Create a list to store the filtered points
    filtered_points = list()

    gen = pc2.read_points(data, skip_nans=True)
    int_data = list(gen)
    y_values = [point[1] for point in int_data]
    y_threshold = max(y_values) * 0.6
    y_threshold2 = min(y_values) * 0.8

    z_values = [point[2] for point in int_data]
    z_threshold = max(z_values) * 0.9
    print("Ythreshold",y_threshold)
    print("Ythreshold2",y_threshold2)

    print("Z_threshold",z_threshold)


    for i, x in enumerate(int_data):
        xc = x[0]
        yc = x[1]
        zc = x[2] 

        newx = np.array(x)
        # temp = -newx[1]
        # newx[1] = -newx[2]
        # newx[2] = temp

        test = x[3] 
        r,g,b = get_rgb(test)
        
        if zc<z_threshold:
            if checkwhite([r,g,b]) or yc<=y_threshold:
                newartificialx = newx
                newartificialx[1] = newartificialx[1]
                filtered_points.append(newartificialx)
                # for i in np.arange(y_threshold2,y_threshold,1):
                #     # artificialx = [xc,i,zc]
                #     # newartificialx = [xc,-zc,-i]
                #     newartificialx = newx
                #     newartificialx[2] = -i    
                #     filtered_points.append(newartificialx)
                pass
            else:
                newartificialx = newx
                newartificialx[1] = newartificialx[1]-200
                filtered_points.append(newartificialx)
            
        # if yc <= y_threshold:
        #     filtered_points.append(newx)
    
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

