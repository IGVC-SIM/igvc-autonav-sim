#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <cmath>

ros::Publisher filtered_point_cloud_pub;
ros::Publisher odom_pub;
double angular_velocity_z = 0.0;
nav_msgs::Odometry latest_odom;

// Hard-coded reference point
const double REF_X = -7.931561996215021;  // Replace with your reference X coordinate
const double REF_Y = 14.314941002522998;  // Replace with your reference Y coordinate
const double DISTANCE_THRESHOLD = 10.0;

const double REF_X2 = -14.127625860234343;  // Replace with your reference X coordinate
const double REF_Y2 = 22.478713612828365;  // Replace with your reference Y coordinate
const double DISTANCE_THRESHOLD2 = 14.0;

bool is_shade_of_green(int r, int g, int b) {
    return ((g > r) && (g > b)) || (r==g && b==g && r<10);
}

bool checkWhite(uint8_t r, uint8_t g, uint8_t b) {
    return (r >= 180 && g >= 180 && b >= 180);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    angular_velocity_z = msg->angular.z;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom = *msg;
}

double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

void filterPointCloud(const sensor_msgs::PointCloud2ConstPtr& input) {
    // Calculate distance from the reference point
    double current_x = latest_odom.pose.pose.position.x;
    double current_y = latest_odom.pose.pose.position.y;
    double distance = calculateDistance(current_x, current_y, REF_X, REF_Y);
    double distance2 = calculateDistance(current_x, current_y, REF_X2, REF_Y2);

    // Only process and publish if angular velocity is below threshold and distance is greater than 5
    if (std::abs(angular_velocity_z) < 0.2 && !(distance < DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD2)) {
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
        float y_threshold2 = 0.8;
        float z_threshold = z_max * 0.9f;

        // Filter points based on thresholds and colors
        for (const auto& point : cloud.points) {
            if (std::isnan(point.z) || std::isnan(point.y)) continue;
            
            uint8_t r = point.r;
            uint8_t g = point.g;
            uint8_t b = point.b;

            if (point.z < z_threshold && !is_shade_of_green(r,g,b)) {
                if (checkWhite(r, g, b) or point.y<y_threshold2)  {
                    filteredCloud.push_back(point);
                }
            }
        }

        // Convert filtered PCL PointCloud back to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(filteredCloud, output);
        output.header = input->header;

        // Publish the filtered point cloud
        filtered_point_cloud_pub.publish(output);

        // Publish the corresponding odometry data
        // latest_odom.header.stamp = input->header.stamp;
        
        // odom_pub.publish(latest_odom);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_rotated");
    ros::NodeHandle nh;

    ros::Subscriber point_cloud_sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 100, filterPointCloud);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 100, cmdVelCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 100, odomCallback);
    filtered_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/altered_point_cloud", 100);
    // odom_pub = nh.advertise<nav_msgs::Odometry>("/filtered_odom", 100);

    ros::spin();
    return 0;
}