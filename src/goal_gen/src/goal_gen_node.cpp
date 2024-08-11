#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>

class MapSearcher
{
public:
    MapSearcher() : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        
        map_sub_ = nh.subscribe("/projected_map", 1, &MapSearcher::mapCallback, this);
        odom_sub_ = nh.subscribe("/odom", 1, &MapSearcher::odomCallback, this);
        
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("/found_block", 10);
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        
        min_goal_distance_ = 2;  // Minimum distance (in meters) for a new goal
    }

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        map_ = *map_msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        if (map_.data.empty()) return;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg->header;
        pose_stamped.pose = odom_msg->pose.pose;

        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
            tf2::doTransform(pose_stamped, bot_pose_map_, transform);
            
            int x = static_cast<int>((bot_pose_map_.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
            int y = static_cast<int>((bot_pose_map_.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
            
            bot_position_ = std::make_pair(x, y);
            
            double roll, pitch, yaw;
            tf2::Quaternion q(
                bot_pose_map_.pose.orientation.x,
                bot_pose_map_.pose.orientation.y,
                bot_pose_map_.pose.orientation.z,
                bot_pose_map_.pose.orientation.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            bot_orientation_ = yaw;
            
            ROS_INFO_STREAM("Bot position in map: (" << bot_position_.first << ", " << bot_position_.second 
                            << "), orientation: " << bot_orientation_);
            
            searchBlock();
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    void searchBlock()
    {
        if (bot_position_.first == -1 || map_.data.empty()) return;

        std::vector<signed char> map_array = map_.data;
        
        int search_radius = static_cast<int>(10 / map_.info.resolution);
        std::pair<double, double> best_goal;
        double best_score = -std::numeric_limits<double>::infinity();
        
        for (int r = search_radius + 5; r <= search_radius - 3; r--)
        {
            for (double theta = -M_PI/2; theta <= M_PI/2; theta += M_PI/40)  // 20 steps in 90-degree arc
            {
                int dx = static_cast<int>(r * cos(bot_orientation_ + theta));
                int dy = static_cast<int>(r * sin(bot_orientation_ + theta));
                
                int x = bot_position_.first + dx;
                int y = bot_position_.second + dy;
                
                if (x >= 0 && x < static_cast<int>(map_.info.width) && 
                    y >= 0 && y < static_cast<int>(map_.info.height))
                {
                    if (map_array[y * map_.info.width + x] == 0 && 
                        isPathClear(bot_position_, std::make_pair(x, y)))
                    {
                        double goal_x = x * map_.info.resolution + map_.info.origin.position.x;
                        double goal_y = y * map_.info.resolution + map_.info.origin.position.y;
                        
                        double score = scoreGoal(goal_x, goal_y);
                        
                        if (score > best_score)
                        {
                            best_score = score;
                            best_goal = std::make_pair(goal_x, goal_y);
                        }
                    }
                }
            }
        }

        if (best_score > -std::numeric_limits<double>::infinity())
        {
            publishMarker(best_goal.first, best_goal.second);
            publishGoal(best_goal.first, best_goal.second);
            visited_areas_.insert(std::make_pair(static_cast<int>(best_goal.first), static_cast<int>(best_goal.second)));
        }
        else
        {
            ROS_INFO("No suitable goal found");
        }
    }

    double scoreGoal(double x, double y)
    {
        double score = 0;
        
        for (const auto& visited : visited_areas_)
        {
            double distance = hypot(x - visited.first, y - visited.second);
            score += std::min(distance, 5.0);  // Cap the benefit at 5 meters
        }
        
        double dx = x - (bot_position_.first * map_.info.resolution + map_.info.origin.position.x);
        double dy = y - (bot_position_.second * map_.info.resolution + map_.info.origin.position.y);
        double angle_to_goal = atan2(dy, dx);
        double angle_diff = std::abs(normalizeAngle(angle_to_goal - bot_orientation_));
        double forward_preference = cos(std::abs(angle_diff-(M_PI/4)));
        score += forward_preference * 10;

        return score;
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    bool isPathClear(const std::pair<int, int>& start, const std::pair<int, int>& end)
    {
        int x0 = start.first, y0 = start.second;
        int x1 = end.first, y1 = end.second;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int x = x0, y = y0;
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (x != x1 || y != y1)
        {
            if (map_.data[y * map_.info.width + x] != 0) return false;
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }

        return true;
    }

    void publishMarker(double x, double y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "found_block";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker_pub_.publish(marker);
    }

    void publishGoal(double x, double y)
    {
        double current_x = bot_position_.first * map_.info.resolution + map_.info.origin.position.x;
        double current_y = bot_position_.second * map_.info.resolution + map_.info.origin.position.y;
        double distance_to_goal = hypot(x - current_x, y - current_y);
        
        if (distance_to_goal < min_goal_distance_)
        {
            ROS_INFO_STREAM("New goal too close to current position. Distance: " << distance_to_goal);
            return;
        }

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, bot_orientation_);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
        
        goal_pub_.publish(goal);
        ROS_INFO_STREAM("Published goal: (" << x << ", " << y << ")");
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher goal_pub_;
    
    nav_msgs::OccupancyGrid map_;
    std::pair<int, int> bot_position_{-1, -1};
    double bot_orientation_ = 0.0;
    geometry_msgs::PoseStamped bot_pose_map_;
    std::set<std::pair<int, int>> visited_areas_;
    double min_goal_distance_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_gen_node");
    MapSearcher map_searcher;
    ros::spin();
    return 0;
}