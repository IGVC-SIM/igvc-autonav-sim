#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber laser_sub;
ros::Publisher laser_pub;


void laserCallback(const sensor_msgs::LaserScan data)
{
    sensor_msgs::LaserScan msg;
    msg = data;
    laser_pub.publish(msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Remapper");

    ros::NodeHandle nh;

    laser_sub = nh.subscribe("/scan",1000,laserCallback);
    laser_pub = nh.advertise<sensor_msgs::LaserScan>("/rtabmap/scan",1000);
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;    
}
