#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odom_callback(msg):
    try:
        # Create a PoseStamped message from the Odometry message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Transform the pose to the map frame
        transformed_pose = tf_buffer.transform(pose, 'map', rospy.Duration(1.0))

        # Publish the transformed pose
        transformed_pose_pub.publish(transformed_pose)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform error: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('odom_to_map_transformer')

    # Create a TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a publisher for the transformed pose
    transformed_pose_pub = rospy.Publisher('/botcoords', PoseStamped, queue_size=10)

    # Subscribe to the /odom topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.spin()
