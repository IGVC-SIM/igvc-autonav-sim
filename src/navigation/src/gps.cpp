#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

ros::Publisher gps_pub;
ros::Subscriber gps_sub;

void gps_callback(sensor_msgs::NavSatFix msg)
{
    msg.header.frame_id = "world";
    gps_pub.publish(msg);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_redirector");
    ros::NodeHandle nh;

    gps_sub = nh.subscribe("/gps", 1, gps_callback);
    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
    while(ros::ok())
        ros::spinOnce();
    return 0;
}
