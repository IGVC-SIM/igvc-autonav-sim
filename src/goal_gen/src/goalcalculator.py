#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import math

class MapSearcher:
    def __init__(self):
        rospy.init_node('map_searcher', anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.marker_pub = rospy.Publisher('/found_block', Marker, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.map = None
        self.bot_position = None
        self.bot_orientation = None

        self.visited_areas = set()
        self.min_goal_distance = 2.0  # Minimum distance (in meters) for a new goal

    def map_callback(self, map_msg):
        self.map = map_msg

    def odom_callback(self, odom_msg):
        if self.map is None:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time())
            bot_pose_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            
            x = int((bot_pose_map.pose.position.x - self.map.info.origin.position.x) / self.map.info.resolution)
            y = int((bot_pose_map.pose.position.y - self.map.info.origin.position.y) / self.map.info.resolution)
            
            self.bot_position = (x, y)
            
            orientation = bot_pose_map.pose.orientation
            _, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self.bot_orientation = yaw
            
            rospy.loginfo(f"Bot position in map: {self.bot_position}, orientation: {self.bot_orientation}")
            
            self.search_block()
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")

    def search_block(self):
        if self.bot_position is None or self.map is None or self.bot_orientation is None:
            return

        map_array = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        
        search_radius = int(10 / self.map.info.resolution)
        best_goal = None
        best_score = float('-inf')
        
        for r in range(search_radius - 2, search_radius + 3):  # Search in a range around 10 units
            for theta in np.linspace(-math.pi/4, math.pi/4, 20):  # 90-degree arc
                dx = int(r * math.cos(self.bot_orientation + theta))
                dy = int(r * math.sin(self.bot_orientation + theta))
                
                x = self.bot_position[0] + dx
                y = self.bot_position[1] + dy
                
                if 0 <= x < self.map.info.width and 0 <= y < self.map.info.height:
                    if map_array[y, x] == 0 and self.is_path_clear(self.bot_position, (x, y)):
                        goal_x = x * self.map.info.resolution + self.map.info.origin.position.x
                        goal_y = y * self.map.info.resolution + self.map.info.origin.position.y
                        
                        # Score the potential goal
                        score = self.score_goal(goal_x, goal_y)
                        
                        if score > best_score:
                            best_score = score
                            best_goal = (goal_x, goal_y)

        if best_goal:
            self.publish_marker(*best_goal)
            self.publish_goal(*best_goal)
            self.visited_areas.add((int(best_goal[0]), int(best_goal[1])))
        else:
            rospy.loginfo("No suitable goal found")

    def score_goal(self, x, y):
        # Higher score is better
        score = 0
        
        # Prefer goals that are farther from visited areas
        for visited_x, visited_y in self.visited_areas:
            distance = math.sqrt((x - visited_x)**2 + (y - visited_y)**2)
            score += min(distance, 5.0)  # Cap the benefit at 5 meters
        
        # Prefer goals that are more in front of the robot
        dx = x - self.bot_position[0] * self.map.info.resolution - self.map.info.origin.position.x
        dy = y - self.bot_position[1] * self.map.info.resolution - self.map.info.origin.position.y
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = abs(self.normalize_angle(angle_to_goal - self.bot_orientation))
        forward_preference = math.cos(angle_diff)  # 1.0 for directly ahead, 0.0 for sideways, -1.0 for behind
        score += forward_preference * 10  # Weight this factor

        return score

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_path_clear(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        map_array = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

        while x != x1 or y != y1:
            if map_array[y, x] != 0:
                return False
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "found_block"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

    def publish_goal(self, x, y):
        # Check if the new goal is far enough from the current position
        current_x = self.bot_position[0] * self.map.info.resolution + self.map.info.origin.position.x
        current_y = self.bot_position[1] * self.map.info.resolution + self.map.info.origin.position.y
        distance_to_goal = math.sqrt((x - current_x)**2 + (y - current_y)**2)
        
        if distance_to_goal < self.min_goal_distance:
            rospy.loginfo(f"New goal too close to current position. Distance: {distance_to_goal}")
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        
        # Set the orientation to face the same direction as the robot
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = math.sin(self.bot_orientation / 2)
        goal.pose.orientation.w = math.cos(self.bot_orientation / 2)
        
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal: {goal}")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
    try:
        MapSearcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass