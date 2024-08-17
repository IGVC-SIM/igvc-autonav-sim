#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import math
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN


class MapSearcher:
    def __init__(self):
        rospy.init_node('map_searcher', anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.map_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.marker_pub = rospy.Publisher('/found_block', Marker, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.map = None
        self.bot_position = None
        self.bot_orientation = None

        self.previous_goals = list()
        self.past_orientations = []
        self.max_orientations = 5

        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_publish)

        self.ramp_goals = [
            (10.981856913385355, 20.638044400288646),
            (-14.127625860234343, 22.478713612828365)
        ]

        self.near_ramp = False
        self.ramp_reached = False

    def check_publish(self, event):
        if self.bot_position is None or self.map is None:
            rospy.loginfo("Not enough information to search and publish yet.")
            return
        # if not self.ramp_reached:
        self.check_near_ramp()
        if len(self.previous_goals) == 0:
            self.get_goal_candidates()
        else:
            dist_left = self.calculate_distance(self.previous_goals[-1], self.convert_to_map_coords(self.bot_position))
            dist_total = self.calculate_distance(self.previous_goals[-1], self.previous_goals[-2])
            if dist_left <= dist_total * 0.7:
                self.get_goal_candidates()
        return

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
            
            # Store the orientation
            self.past_orientations.append(yaw)
            if len(self.past_orientations) > self.max_orientations:
                self.past_orientations.pop(0)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")


    def cluster_points_in_grid(self, grid, eps=6, min_samples=6):
        points = np.argwhere(grid > 0)
        points = points[:, [1, 0]]  # Swap columns to get (x, y) format
        print("Map shape:", points.shape)

        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        dbscan.fit(points)
        
        labels = dbscan.labels_
        unique_labels = np.unique(labels[labels != -1])
        clusters = [points[labels == label] for label in unique_labels]
        clusters.sort(key=len, reverse=True)
        largest_clusters = clusters[:2]
        
        print("Number of clusters found:", len(clusters))
        
        while len(largest_clusters) < 2:
            largest_clusters.append(np.array([]))

        print(f"Number of points in largest cluster: {len(largest_clusters[0])}")
        print(f"Number of points in second largest cluster: {len(largest_clusters[1])}")

        return largest_clusters[0], largest_clusters[1], points
        

    def convert_to_map_coords(self, coord):
        origin = np.array([self.map.info.origin.position.x, self.map.info.origin.position.y])
        newcoord = origin + np.array([x*self.map.info.resolution for x in coord])
        return newcoord


    def find_midpoints_of_nearest_pairs(self, set1, set2, all_points, reference_point, min_distance, max_distance):
        # Ensure inputs are numpy arrays
        set1 = np.array(set1)
        set2 = np.array(set2)
        reference_point = np.array(reference_point)

        # Build KD-trees for both sets
        tree1 = KDTree(set1)
        tree2 = KDTree(set2)

        # Find nearest points in set2 for each point in set1
        distances1, indices1 = tree1.query(set2)
        
        # Find nearest points in set1 for each point in set2
        distances2, indices2 = tree2.query(set1)

        # Collect all pairs
        pairs = []
        for i, (dist, idx) in enumerate(zip(distances1, indices1)):
            pairs.append((set2[i], set1[idx], dist))
        for i, (dist, idx) in enumerate(zip(distances2, indices2)):
            pairs.append((set1[i], set2[idx], dist))

        # Calculate midpoints and their scores
        midpoints = []
        for p1, p2, nearest_pt_dist in pairs:
            midpoint = (p1 + p2) / 2
            distance_to_reference = np.linalg.norm(midpoint - reference_point)
            if min_distance <= distance_to_reference <= max_distance:
                min_distance_to_points = np.min(np.linalg.norm(all_points - midpoint, axis=1))
                angle = self.calculate_angle(self.convert_to_map_coords(midpoint), self.previous_goals[-1], self.previous_goals[-2])
                if angle > 93:
                    # Combine both criteria: distance from reference point and distance from all points
                    score = ((distance_to_reference**2)+(min_distance_to_points**1.7)+(nearest_pt_dist**2))*(angle**(1/2.5))                    
                    midpoints.append((midpoint, score))

        # Sort midpoints by combined score (higher is better)
        midpoints.sort(key=lambda x: x[1], reverse=True)

        # Return only the sorted midpoints as an array of (x,y) tuples
        return np.array([(m[0][0], m[0][1]) for m in midpoints])


    def is_point_on_line(self, point_set, point1, point2):
        def bresenham_line(x0, y0, x1, y1):
            points = set()
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            x, y = x0, y0
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            
            if dx > dy:
                err = dx / 2.0
                while x != x1:
                    points.add((x, y))
                    err -= dy
                    if err < 0:
                        y += sy
                        err += dx
                    x += sx
            else:
                err = dy / 2.0
                while y != y1:
                    points.add((x, y))
                    err -= dx
                    if err < 0:
                        x += sx
                        err += dy
                    y += sy
            
            points.add((x, y))
            return points

        x1, y1 = map(int, point1)
        x2, y2 = map(int, point2)
        
        line_points = bresenham_line(x1, y1, x2, y2)
        
        for point in point_set:
            if tuple(map(int, point)) in line_points:
                return True
        return False



    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

    def check_near_ramp(self):
        # for i, goal in enumerate(self.ramp_goals[:-1]):
        #     dist = self.calculate_distance(self.convert_to_map_coords(self.bot_position), goal)
        #     if dist <= 10:
        #         return i
        if self.calculate_distance(self.convert_to_map_coords(self.bot_position), self.ramp_goals[0]) < 7:
            self.near_ramp = True
            rospy.loginfo("Near ramp")
        else:
            rospy.loginfo("Not near ramp")
            self.near_ramp = False
        if self.ramp_reached is False and self.calculate_distance(self.convert_to_map_coords(self.bot_position), self.ramp_goals[-1]) < 1:
            # self.previous_goals.append(self.convert_to_map_coords(self.bot_position))
            # self.previous_goals.append(self.ramp_goals[-1])
            rospy.loginfo("Ramp traversed")
            self.ramp_reached = True
            self.near_ramp = False
            self.previous_goals.clear()
    
    def calculate_angle(self, candidate_goal, previous_goal, second_previous_goal):
        # Convert goals to numpy arrays for easier vector operations
        v1 = np.array(candidate_goal) - np.array(previous_goal)
        v2 = np.array(second_previous_goal) - np.array(previous_goal)
        
        # Calculate the angle between the two vectors
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        
        # Convert angle from radians to degrees
        angle_degrees = np.degrees(angle)

        return angle_degrees


    def get_goal_candidates(self):
        if self.bot_position is None or self.map is None:
            rospy.loginfo("Not enough information to search and publish yet.")
            return
        map_array = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        print("Map shape: ", map_array.shape)

        # if self.near_ramp != False:
        #     rospy.loginfo("Ramp mode.")
        #     can_publish = False
        #     # self.publish_goal(self.ramp_goals[near_ramp])
        if len(self.previous_goals)==0:
            can_publish = True
            self.previous_goals.append(self.convert_to_map_coords(self.bot_position))
            if not self.ramp_reached:
                desired_point = (self.bot_position[0]+3/self.map.info.resolution,self.bot_position[1])
            else:
                desired_point = (self.bot_position[0]-3/self.map.info.resolution,self.bot_position[1])
        else:
            can_publish = False
            set1, set2, all_points = self.cluster_points_in_grid(map_array)
            if len(set1) > 200 and len(set2) > 200:
                midpoints = self.find_midpoints_of_nearest_pairs(set1, set2, all_points, np.array(self.bot_position), 4/self.map.info.resolution, 7/self.map.info.resolution)
                for midpoint in midpoints:
                    value = map_array[int(midpoint[1]),int(midpoint[0])]
                    if value == 0:
                        desired_point = midpoint
                        can_publish = True
            else:
                rospy.loginfo("No sets found with point lengths larger than 200. Set lengths: "+str(len(set1))+", "+str(len(set2)))

        if can_publish == True:
            print("Map array goal: ", desired_point)
            x, y = desired_point[0], desired_point[1]
            goal_x, goal_y = self.convert_to_map_coords((x, y))
            print("Goal: ",goal_x, goal_y)
            self.publish_marker(goal_x,goal_y)
            self.publish_goal(goal_x, goal_y)
            self.previous_goals.append((goal_x,goal_y))



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


    def calculate_average_orientation(self):
        if not self.past_orientations:
            return self.bot_orientation

        # Calculate average orientation
        sin_sum = sum(math.sin(o) for o in self.past_orientations)
        cos_sum = sum(math.cos(o) for o in self.past_orientations)
        avg_orientation = math.atan2(sin_sum, cos_sum)
        
        return avg_orientation


    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        
        # Use the average orientation
        avg_orientation = self.calculate_average_orientation()
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = math.sin(avg_orientation / 2)
        goal.pose.orientation.w = math.cos(avg_orientation / 2)

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