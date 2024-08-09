#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import ctypes
import sensor_msgs.point_cloud2 as pc2


def unpack_rgb(packed_color):
    """Unpack RGB values from a packed float."""
    rgb_array = np.zeros((packed_color.size, 3), dtype=np.uint8)
    rgb_array[:, 0] = (packed_color >> 16) & 0xFF  # Red
    rgb_array[:, 1] = (packed_color >> 8) & 0xFF   # Green
    rgb_array[:, 2] = packed_color & 0xFF          # Blue
    return rgb_array


def is_white(rgb):
    """Check if RGB values represent white."""
    return np.all(rgb > 180)

def filter_point_cloud(data):
    # Convert PointCloud2 to numpy array
    gen = pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = np.array(list(gen), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])

    # Calculate thresholds
    y_threshold = np.max(points['y']) * 0.6
    z_threshold = np.max(points['z']) * 0.9

    # Create mask for z threshold
    z_mask = points['z'] < z_threshold

    # Unpack RGB values
    rgb_values = unpack_rgb(points['rgb'].view(np.uint32))

    # Create white mask
    white_mask = np.all(rgb_values > 100, axis=1)

    # Create y threshold mask
    y_mask = points['y'] <= y_threshold

    # Combine masks
    final_mask = z_mask & (white_mask | y_mask)

    # Apply mask to points
    filtered_points = points[final_mask].copy()

    # Adjust y values for non-white points
    non_white_mask = ~white_mask[final_mask]
    filtered_points[non_white_mask]

    # Create a new PointCloud2 message for the filtered points
    header = data.header
    fields = data.fields
    new_cloud = pc2.create_cloud(header, fields, filtered_points)

    # Publish the filtered point cloud
    filtered_point_cloud_pub.publish(new_cloud)


if __name__ == "__main__":
    rospy.init_node('filternode')

    # Subscribe to the input PointCloud2 topic
    point_cloud_sub = rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, filter_point_cloud)

    # Create a publisher for the filtered PointCloud2 topic
    filtered_point_cloud_pub = rospy.Publisher('/altered_point_cloud', PointCloud2, queue_size=1)

    rospy.spin()