#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
ros::Publisher filtered_point_cloud_pub;

bool checkWhite(uint8_t r, uint8_t g, uint8_t b) {
    return (r >= 180 && g >= 180 && b >= 180);
}

void filterPointCloud(const sensor_msgs::PointCloud2ConstPtr& input) {
    // Convert the ROS message to a PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);

    pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;
    
    // Find thresholds for filtering
    float y_max = -std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
 
    for (const auto& point : cloud.points) {
        if (!std::isnan(point.y)) {
            if (point.y > y_max) y_max = point.y;
            if (point.y < y_min) y_min = point.y;
        }
        if (!std::isnan(point.z) && point.z > z_max) z_max = point.z;
    }

    float y_threshold = y_max * 0.6f;
    float y_threshold2 = y_min * 0.8f;
    float z_threshold = z_max * 0.9f;

    // Filter points based on thresholds and colors
    for (const auto& point : cloud.points) {
        if (std::isnan(point.z) || std::isnan(point.y)) continue;
        
        uint8_t r = point.r;
        uint8_t g = point.g;
        uint8_t b = point.b;

        if (point.z < z_threshold) {
            if (checkWhite(r, g, b))  {
                filteredCloud.push_back(point);
            } else {
                // pcl::PointXYZRGB newPoint = point;
                // newPoint.y -= 200;
                // filteredCloud.push_back(newPoint);
            }
        }
    }

    // Convert filtered PCL PointCloud back to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filteredCloud, output);
    output.header = input->header;

    // Publish the filtered point cloud
    filtered_point_cloud_pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_rotated");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 100, filterPointCloud);
    filtered_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/altered_point_cloud", 100);

    ros::spin();
    return 0;
}