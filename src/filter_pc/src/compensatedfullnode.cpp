#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class MotionCompensatedOctomap {
public:
    MotionCompensatedOctomap() {
        sub_ = nh_.subscribe("/altered_point_cloud", 1, &MotionCompensatedOctomap::cloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/compensated_cloud", 1);
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        geometry_msgs::TransformStamped transform;
        try {
            transform = tfBuffer_.lookupTransform("odom", cloud_msg->header.frame_id,
                                                  cloud_msg->header.stamp, ros::Duration(0.1));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        sensor_msgs::PointCloud2 cloud_out;
        tf2::doTransform(*cloud_msg, cloud_out, transform);

        pub_.publish(cloud_out);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_compensated_octomap");
    MotionCompensatedOctomap compensator;
    ros::spin();
    return 0;
}