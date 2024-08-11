#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <nanoflann.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>

class MapSearcher {
public:
    MapSearcher() : nh_("~") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = nh_.subscribe("/projected_map", 1, &MapSearcher::mapCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &MapSearcher::odomCallback, this);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/found_block", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        timer_ = nh_.createTimer(ros::Duration(3.0), &MapSearcher::getGoalCandidates, this);

        max_orientations_ = 7;
    }

private:
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher goal_pub_;
    ros::Timer timer_;

    nav_msgs::OccupancyGrid::ConstPtr map_;
    std::pair<int, int> bot_position_;
    double bot_orientation_;

    std::vector<std::pair<double, double>> previous_goals_;
    std::vector<double> past_orientations_;
    int max_orientations_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        map_ = map_msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        if (!map_) {
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg->header;
        pose_stamped.pose = odom_msg->pose.pose;

        try {
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform("map", "odom", ros::Time(0));
            geometry_msgs::PoseStamped bot_pose_map;
            tf2::doTransform(pose_stamped, bot_pose_map, transform);

            int x = static_cast<int>((bot_pose_map.pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
            int y = static_cast<int>((bot_pose_map.pose.position.y - map_->info.origin.position.y) / map_->info.resolution);

            bot_position_ = std::make_pair(x, y);

            tf2::Quaternion q(
                bot_pose_map.pose.orientation.x,
                bot_pose_map.pose.orientation.y,
                bot_pose_map.pose.orientation.z,
                bot_pose_map.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            bot_orientation_ = yaw;

            past_orientations_.push_back(yaw);
            if (past_orientations_.size() > max_orientations_) {
                past_orientations_.erase(past_orientations_.begin());
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> clusterPointsInGrid(const std::vector<int8_t>& grid, int width, int height, double eps = 5, size_t min_samples = 2) {
        std::vector<Eigen::Vector2d> points;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (grid[y * width + x] > 0) {
                    points.emplace_back(x, y);
                }
            }
        }

        mlpack::DBSCAN<> dbscan(eps, min_samples);
        arma::mat data(2, points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            data(0, i) = points[i](0);
            data(1, i) = points[i](1);
        }

        arma::Row<size_t> assignments;
        dbscan.Cluster(data, assignments);

        std::vector<std::vector<Eigen::Vector2d>> clusters;
        for (size_t i = 0; i < assignments.n_elem; ++i) {
            if (assignments[i] != SIZE_MAX) {
                if (clusters.size() <= assignments[i]) {
                    clusters.resize(assignments[i] + 1);
                }
                clusters[assignments[i]].emplace_back(data(0, i), data(1, i));
            }
        }

        std::sort(clusters.begin(), clusters.end(), 
                  [](const auto& a, const auto& b) { return a.size() > b.size(); });

        Eigen::MatrixXd largest_cluster, second_largest_cluster;
        if (clusters.size() > 0) {
            largest_cluster = Eigen::MatrixXd(clusters[0].size(), 2);
            for (size_t i = 0; i < clusters[0].size(); ++i) {
                largest_cluster.row(i) = clusters[0][i];
            }
        }
        if (clusters.size() > 1) {
            second_largest_cluster = Eigen::MatrixXd(clusters[1].size(), 2);
            for (size_t i = 0; i < clusters[1].size(); ++i) {
                second_largest_cluster.row(i) = clusters[1][i];
            }
        }

        return {largest_cluster, second_largest_cluster};
    }

    Eigen::MatrixXd findMidpointsOfNearestPairs(const Eigen::MatrixXd& set1, const Eigen::MatrixXd& set2, 
                                                const Eigen::Vector2d& reference_point, double min_distance, double max_distance) {
        using KDTreeType = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 2, nanoflann::metric_L2>;
        KDTreeType tree1(2, set1, 10);
        KDTreeType tree2(2, set2, 10);

        std::vector<std::pair<Eigen::Vector2d, double>> midpoints;
        for (int i = 0; i < set1.rows(); ++i) {
            std::vector<size_t> ret_index(1);
            std::vector<double> out_dist_sqr(1);
            tree2.index->knnSearch(set1.row(i).data(), 1, &ret_index[0], &out_dist_sqr[0]);
            
            Eigen::Vector2d midpoint = (set1.row(i) + set2.row(ret_index[0])) / 2.0;
            double distance_to_reference = (midpoint - reference_point).norm();
            
            if (min_distance <= distance_to_reference && distance_to_reference <= max_distance) {
                double min_distance_to_points = std::min(
                    (set1.rowwise() - midpoint.transpose()).rowwise().norm().minCoeff(),
                    (set2.rowwise() - midpoint.transpose()).rowwise().norm().minCoeff()
                );
                double score = distance_to_reference * min_distance_to_points;
                midpoints.emplace_back(midpoint, score);
            }
        }

        std::sort(midpoints.begin(), midpoints.end(), 
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        Eigen::MatrixXd result(midpoints.size(), 2);
        for (size_t i = 0; i < midpoints.size(); ++i) {
            result.row(i) = midpoints[i].first;
        }

        return result;
    }

    double calculateDistance(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2) {
        return (point1 - point2).norm();
    }

    bool validateGoalDirection(const Eigen::Vector2d& candidate_goal, 
                               const Eigen::Vector2d& previous_goal, 
                               const Eigen::Vector2d& second_previous_goal) {
        Eigen::Vector2d v1 = candidate_goal - previous_goal;
        Eigen::Vector2d v2 = previous_goal - second_previous_goal;
        
        double cos_angle = v1.dot(v2) / (v1.norm() * v2.norm());
        double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
        
        double angle_degrees = angle * 180.0 / M_PI;
        return angle_degrees <= 70;
    }

    void getGoalCandidates(const ros::TimerEvent& event) {
        if (bot_position_.first == 0 && bot_position_.second == 0 || !map_) {
            ROS_INFO("Not enough information to search and publish yet.");
            return;
        }

        Eigen::MatrixXi map_array = Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            map_->data.data(), map_->info.height, map_->info.width).cast<int>();

        auto [set1, set2] = clusterPointsInGrid(map_->data, map_->info.width, map_->info.height);
        
        Eigen::MatrixXd midpoints = findMidpointsOfNearestPairs(
            set1, set2, Eigen::Vector2d(bot_position_.first, bot_position_.second), 
            3 / map_->info.resolution, 7 / map_->info.resolution);

        bool can_publish = false;
        Eigen::Vector2d desired_point;

        if (previous_goals_.empty()) {
            can_publish = true;
            desired_point = Eigen::Vector2d(bot_position_.first + 0.5 / map_->info.resolution, bot_position_.second);
        } else if (previous_goals_.size() == 1) {
            can_publish = true;
            desired_point = Eigen::Vector2d(bot_position_.first + 0.5 / map_->info.resolution, bot_position_.second);
        } else {
            for (int i = 0; i < midpoints.rows(); ++i) {
                Eigen::Vector2d midpoint = midpoints.row(i);
                int value = map_array(static_cast<int>(midpoint.y()), static_cast<int>(midpoint.x()));
                if (value == 0) {
                    double goal_x = midpoint.x() * map_->info.resolution + map_->info.origin.position.x;
                    double goal_y = midpoint.y() * map_->info.resolution + map_->info.origin.position.y;
                    
                    can_publish = validateGoalDirection(
                        Eigen::Vector2d(goal_x, goal_y),
                        Eigen::Vector2d(previous_goals_.back().first, previous_goals_.back().second),
                        Eigen::Vector2d(previous_goals_[previous_goals_.size() - 2].first, 
                                        previous_goals_[previous_goals_.size() - 2].second));
                    
                    if (can_publish) {
                        ROS_INFO("Lies within angle.");
                        desired_point = midpoint;
                        break;
                    } else {
                        ROS_INFO("Does not lie within angle.");
                    }
                }
            }
        }

        if (can_publish) {
            double goal_x = desired_point.x() * map_->info.resolution + map_->info.origin.position.x;
            double goal_y = desired_point.y() * map_->info.resolution + map_->info.origin.position.y;
            publishMarker(goal_x, goal_y);
            publishGoal(goal_x, goal_y);
        }
    }

    void publishMarker(double x, double y) {
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
        
        marker_pub_.publish(marker);
    }

    double calculateAverageOrientation() {
        if (past_orientations_.empty()) {
            return 0.0;
        }
        
        double sin_sum = 0.0, cos_sum = 0.0;
        for (double o : past_orientations_) {
            sin_sum += std::sin(o);
            cos_sum += std::cos(o);
        }
        return std::atan2(sin_sum, cos_sum);
    }

    void publishGoal(double x, double y) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0;
        
        double avg_orientation = calculateAverageOrientation();
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = std::sin(avg_orientation / 2);
        goal.pose.orientation.w = std::cos(avg_orientation / 2);

        goal_pub_.publish(goal);
        ROS_INFO_STREAM("Published goal: " << goal);

        previous_goals_.emplace_back(x, y);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_searcher");
    MapSearcher map_searcher;
    ros::spin();
    return 0;
}