#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

class ImprovedDynamicCostmapNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher costmap_pub_;
    
    nav_msgs::OccupancyGrid costmap_;
    std::vector<double> log_odds_map_;
    double resolution_;
    int width_, height_;
    double min_z_, max_z_;
    double hit_log_odds_, miss_log_odds_;
    double occupancy_threshold_, probability_hit_, probability_miss_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string global_frame_, sensor_frame_;

public:
    ImprovedDynamicCostmapNode() : nh_("~"), tf_listener_(tf_buffer_)
    {
        // Get parameters
        nh_.param("resolution", resolution_, 0.05);
        nh_.param("width", width_, 2000);
        nh_.param("height", height_, 2000);
        nh_.param("min_z", min_z_, 0.1);
        nh_.param("max_z", max_z_, 2.0);
        nh_.param("probability_hit", probability_hit_, 0.7);
        nh_.param("probability_miss", probability_miss_, 0.4);
        nh_.param("occupancy_threshold", occupancy_threshold_, 0.5);
        nh_.param<std::string>("global_frame", global_frame_, "map");
        nh_.param<std::string>("sensor_frame", sensor_frame_, "base_link");

        hit_log_odds_ = std::log(probability_hit_ / (1 - probability_hit_));
        miss_log_odds_ = std::log(probability_miss_ / (1 - probability_miss_));

        // Initialize costmap and log-odds map
        costmap_.info.resolution = resolution_;
        costmap_.info.width = width_;
        costmap_.info.height = height_;
        costmap_.info.origin.position.x = -width_ * resolution_ / 2;
        costmap_.info.origin.position.y = -height_ * resolution_ / 2;
        costmap_.data.resize(width_ * height_, 50);  // Initialize with unknown (50)
        log_odds_map_.resize(width_ * height_, 0.0);

        // Set up subscriber and publisher
        cloud_sub_ = nh_.subscribe("/altered_point_cloud", 1, &ImprovedDynamicCostmapNode::cloudCallback, this);
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/costmap", 1, true);  // Add latching

        ROS_INFO("ImprovedDynamicCostmapNode initialized with %dx%d grid", width_, height_);
    }

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        ROS_INFO("Received point cloud with %d points, frame_id: %s, stamp: %.3f", 
                cloud_msg->width * cloud_msg->height, cloud_msg->header.frame_id.c_str(), 
                cloud_msg->header.stamp.toSec());

        if (cloud_msg->width * cloud_msg->height == 0) {
            ROS_WARN("Received an empty point cloud. Skipping update.");
            return;
        }

        sensor_msgs::PointCloud2 transformed_cloud;
        try {
            // First, try to transform to the global frame
            tf_buffer_.transform(*cloud_msg, transformed_cloud, global_frame_, ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform point cloud from %s to %s: %s", 
                    cloud_msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
            
            // If transformation fails, try to use the original cloud
            ROS_WARN("Attempting to use untransformed cloud...");
            transformed_cloud = *cloud_msg;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *cloud);

        ROS_INFO("PCL cloud has %zu points", cloud->points.size());

        // Reset costmap
        std::fill(costmap_.data.begin(), costmap_.data.end(), 50);

        // Update costmap
        int points_processed = 0;
        int points_out_of_bounds = 0;
        for (const auto& point : cloud->points)
        {
            int x = (point.x - costmap_.info.origin.position.x) / resolution_;
            int y = (point.y - costmap_.info.origin.position.y) / resolution_;

            if (x >= 0 && x < width_ && y >= 0 && y < height_)
            {
                int index = y * width_ + x;
                costmap_.data[index] = 100;  // Mark as occupied
                points_processed++;
            }
            else
            {
                points_out_of_bounds++;
            }
        }

        ROS_INFO("Processed %d points, %d points were out of bounds", points_processed, points_out_of_bounds);
        ROS_INFO("Costmap origin: (%.2f, %.2f), size: %dx%d, resolution: %.2f", 
                costmap_.info.origin.position.x, costmap_.info.origin.position.y, 
                width_, height_, resolution_);
        
        // Print some sample points
        if (!cloud->points.empty()) {
            ROS_INFO("Sample points:");
            for (int i = 0; i < std::min(5, (int)cloud->points.size()); ++i) {
                ROS_INFO("  Point %d: (%.2f, %.2f, %.2f)", i, 
                        cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            }
        }

        // Publish costmap
        costmap_.header = transformed_cloud.header;
        costmap_pub_.publish(costmap_);

        ROS_INFO("Published costmap");
    }

    void updateLogOdds(int x, int y, bool hit)
    {
        int index = y * width_ + x;
        log_odds_map_[index] += hit ? hit_log_odds_ : miss_log_odds_;
        log_odds_map_[index] = std::max(-100.0, std::min(100.0, log_odds_map_[index]));
    }

    void raytrace(const Eigen::Vector3d& start, const Eigen::Vector3d& end)
    {
        Eigen::Vector3d direction = end - start;
        double length = direction.norm();
        direction.normalize();

        for (double d = 0; d < length; d += resolution_)
        {
            Eigen::Vector3d p = start + d * direction;
            int x = (p.x() - costmap_.info.origin.position.x) / resolution_;
            int y = (p.y() - costmap_.info.origin.position.y) / resolution_;

            if (x >= 0 && x < width_ && y >= 0 && y < height_)
            {
                updateLogOdds(x, y, false);
            }
        }
    }

    void updateCostmap()
    {
        int occupied_cells = 0;
        int free_cells = 0;
        int unknown_cells = 0;

        for (size_t i = 0; i < log_odds_map_.size(); ++i)
        {
            double probability = 1.0 - (1.0 / (1.0 + std::exp(log_odds_map_[i])));
            if (probability > occupancy_threshold_) {
                costmap_.data[i] = 100;
                occupied_cells++;
            } else if (probability < (1 - occupancy_threshold_)) {
                costmap_.data[i] = 0;
                free_cells++;
            } else {
                costmap_.data[i] = -1;
                unknown_cells++;
            }
        }

        ROS_INFO("Costmap update: %d occupied, %d free, %d unknown cells", 
                 occupied_cells, free_cells, unknown_cells);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "improved_dynamic_costmap_node");
    ImprovedDynamicCostmapNode node;
    ros::spin();
    return 0;
}