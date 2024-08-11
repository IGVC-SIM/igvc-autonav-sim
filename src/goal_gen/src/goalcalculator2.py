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
        self.max_orientations = 7

        self.timer = rospy.Timer(rospy.Duration(3), self.get_goal_candidates)

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

    def cluster_points_in_grid(self, grid, eps=5, min_samples=2):
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
        print(f"Number of points in largest cluster: {len(largest_clusters[0])}")
        print(f"Number of points in second largest cluster: {len(largest_clusters[1])}")
        
        while len(largest_clusters) < 2:
            largest_clusters.append(np.array([]))

        return largest_clusters[0], largest_clusters[1]

    # def find_midpoints_of_nearest_pairs(self, set1, set2, reference_point, min_distance, max_distance):
    #     set1 = np.array(set1)
    #     set2 = np.array(set2)
    #     reference_point = np.array(reference_point)

    #     tree1 = KDTree(set1)
    #     tree2 = KDTree(set2)

    #     distances1, indices1 = tree1.query(set2)
    #     distances2, indices2 = tree2.query(set1)

    #     pairs = []
    #     for i, (dist, idx) in enumerate(zip(distances1, indices1)):
    #         pairs.append((set2[i], set1[idx], dist))
    #     for i, (dist, idx) in enumerate(zip(distances2, indices2)):
    #         pairs.append((set1[i], set2[idx], dist))

    #     midpoints = []
    #     for p1, p2, _ in pairs:
    #         midpoint = (p1 + p2) / 2
    #         distance_to_reference = np.linalg.norm(midpoint - reference_point)
    #         if min_distance <= distance_to_reference <= max_distance:
    #             min_distance_to_points = min(
    #                 np.min(np.linalg.norm(set1 - midpoint, axis=1)),
    #                 np.min(np.linalg.norm(set2 - midpoint, axis=1))
    #             )
    #             midpoints.append((midpoint, min_distance_to_points))

    #     midpoints.sort(key=lambda x: x[1], reverse=True)

    #     return np.array([(m[0][0], m[0][1]) for m in midpoints])

    # def find_midpoints_of_nearest_pairs(self, set1, set2, reference_point, min_distance, max_distance):
    #     # Ensure inputs are numpy arrays
    #     set1 = np.array(set1)
    #     set2 = np.array(set2)
    #     reference_point = np.array(reference_point)

    #     # Build KD-trees for both sets
    #     tree1 = KDTree(set1)
    #     tree2 = KDTree(set2)

    #     # Find nearest points in set2 for each point in set1
    #     distances1, indices1 = tree1.query(set2)
        
    #     # Find nearest points in set1 for each point in set2
    #     distances2, indices2 = tree2.query(set1)

    #     # Collect all pairs
    #     pairs = []
    #     for i, (dist, idx) in enumerate(zip(distances1, indices1)):
    #         pairs.append((set2[i], set1[idx], dist))
    #     for i, (dist, idx) in enumerate(zip(distances2, indices2)):
    #         pairs.append((set1[i], set2[idx], dist))

    #     # Calculate midpoints
    #     midpoints = []
    #     for p1, p2, _ in pairs:
    #         midpoint = (p1 + p2) / 2
    #         distance_to_reference = np.linalg.norm(midpoint - reference_point)
    #         if min_distance <= distance_to_reference <= max_distance:
    #             midpoints.append((midpoint, distance_to_reference))

    #     # Sort midpoints by distance to reference point
    #     midpoints.sort(key=lambda x: x[1], reverse=True)

    #     # Return only the sorted midpoints within the specified range as an array of (x,y) tuples
    #     return np.array([(m[0][0], m[0][1]) for m in midpoints])

    def find_midpoints_of_nearest_pairs(self, set1, set2, reference_point, min_distance, max_distance):
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
        for p1, p2, _ in pairs:
            midpoint = (p1 + p2) / 2
            distance_to_reference = np.linalg.norm(midpoint - reference_point)
            if min_distance <= distance_to_reference <= max_distance:
                min_distance_to_points = min(
                    np.min(np.linalg.norm(set1 - midpoint, axis=1)),
                    np.min(np.linalg.norm(set2 - midpoint, axis=1))
                )
                # Combine both criteria: distance from reference point and distance from all points
                score = math.sqrt(distance_to_reference) * min_distance_to_points
                midpoints.append((midpoint, score))

        # Sort midpoints by combined score (higher is better)
        midpoints.sort(key=lambda x: x[1], reverse=True)

        # Return only the sorted midpoints as an array of (x,y) tuples
        return np.array([(m[0][0], m[0][1]) for m in midpoints])

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

    # def validate_goal_direction(self, candidate_goal):
    #     # increasing_distance = True
    #     # for i in range(len(self.previous_goals)-1, -1, -1):
    #     #     compared_goal = self.previous_goals[i]
    #     #     dist2 = self.calculate_distance(candidate_goal,compared_goal)
    #     #     dist1 = self.calculate_distance(candidate_goal,self.previous_goals[i-1])
    #     #     if (dist2<dist1) or dist2<3.5:
    #     #         increasing_distance = False
    #     #         break
    #     candidate_goal = [x/self.map.info.resolution for x in candidate_goal]
    #     if len(self.previous_goals) == 2:
    #         distances = [self.calculate_distance(candidate_goal, compared_goal) for compared_goal in self.previous_goals]
    #         temp = distances[-1]
    #         distances[-1] = self.calculate_distance(candidate_goal, [x/self.map.info.resolution for x in self.bot_position])
    #         distances.append(temp)
    #         if min(distances) == distances[-1]:
    #             return True
    #         else:
    #             return False
    #     else:
    #         return True
    def validate_goal_direction(self, candidate_goal, previous_goal, second_previous_goal):
        # Convert goals to numpy arrays for easier vector operations
        v1 = np.array(candidate_goal) - np.array(previous_goal)
        v2 = np.array(previous_goal) - np.array(second_previous_goal)
        
        # Calculate the angle between the two vectors
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        
        # Convert angle from radians to degrees
        angle_degrees = np.degrees(angle)
        print(angle_degrees)
        # Check if the angle is greater than or equal to 100 degrees
        return angle_degrees <= 70

    def get_goal_candidates(self, event):
        if self.bot_position is None or self.map is None:
            rospy.loginfo("Not enough information to search and publish yet.")
            return

        map_array = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        print("Map shape: ", map_array.shape)
        set1, set2 = self.cluster_points_in_grid(map_array)
        midpoints = self.find_midpoints_of_nearest_pairs(set1, set2, np.array(self.bot_position), 2/self.map.info.resolution, 10/self.map.info.resolution)
        if len(self.previous_goals)==0:
            can_publish = True
            desired_point = (self.bot_position[0]+1/self.map.info.resolution,self.bot_position[1])
        elif len(self.previous_goals)==1:
            can_publish = True
            desired_point = (self.bot_position[0]+1/self.map.info.resolution,self.bot_position[1])
        else:
            can_publish = False
            for midpoint in midpoints:
                value = map_array[int(midpoint[1]),int(midpoint[0])]
                if value == 0:
                    x, y = midpoint
                    bot_x = self.bot_position[0] * self.map.info.resolution + self.map.info.origin.position.x
                    bot_y = self.bot_position[1] * self.map.info.resolution + self.map.info.origin.position.y
                    goal_x = x * self.map.info.resolution + self.map.info.origin.position.x
                    goal_y = y * self.map.info.resolution + self.map.info.origin.position.y
                    # distance_bot_current_goal = math.sqrt((goal_x - bot_x)**2 + (goal_y - bot_y)**2)
                    # can_publish = True
                    can_publish = self.validate_goal_direction((goal_x, goal_y), self.previous_goals[-1], self.previous_goals[-2])
                    print(can_publish)
                    if can_publish == True:
                        rospy.loginfo("Lies within angle.")
                        desired_point = midpoint
                        break
                    else:
                        rospy.loginfo("Does not lie within angle.")
                    # for prevgoal in self.previous_goals[:-1]:
                    #     prevx, prevy = prevgoal
                    #     distance_prev_current_goal = math.sqrt((prevx-goal_x)**2 + (prevy-goal_y)**2)
                    #     if distance_bot_current_goal > distance_prev_current_goal:
                    #         print("Distance of bot from current goal is greater than distance between current goal and one of previous goals.")
                    #         can_publish = False

        if can_publish == True:
            print("Map array goal: ", desired_point)
            x, y = desired_point[0], desired_point[1]
            goal_x = x * self.map.info.resolution + self.map.info.origin.position.x
            goal_y = y * self.map.info.resolution + self.map.info.origin.position.y
            print("Goal: ",goal_x, goal_y)
            self.publish_marker(goal_x,goal_y)
            self.publish_goal(goal_x, goal_y)
            can_publish = False

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
            return 0.0  # Default to 0 if no orientations are available
        
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

        # if len(self.previous_goals) > 0:
        #     distance = math.sqrt((x-self.previous_goals[-1][0])**2 + (y-self.previous_goals[-1][1])**2)
        # else:
        #     distance = 5

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal: {goal}")

        self.previous_goals.append((x,y))

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