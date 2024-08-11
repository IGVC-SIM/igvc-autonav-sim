#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>


class GoalPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Subscriber odom_sub_;
    std::vector<std::vector<double>> goals_;
    std::vector<double> course_exit_,curr_pos = {0.0,0.0};
    int current_goal_index_;


public:
    GoalPublisher() : nh_("~"), current_goal_index_(0)
    {
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom",1,&GoalPublisher::odomCallback,this);
        // Initialize goals (replace these with your actual map coordinates)
        goals_ = {{-15.286769536694717, 22.865746635651142, -0.0009297148376025083,-0.002186954408046168, 0.47346301740409347,-0.8808104926778229},
            {-5.927406361436967,9.822598731074036, -9.189294660504249e-05,-0.0027969263674667627,0.00023919738084792402,-0.9999960557638187},
            {5.242101677453615,10.25232650192324, -0.00010199060590320767,-0.0024030102656244865, 0.04316156223982357,-0.9990652104265254},
            {12.899319957901728,21.778561517673367, 3.3468186372837667e-05,-0.0022263573359656555,-0.005320117068503288,-0.9999833691453426}};
        //COUrse exit,Ramp ENtry, Ramp Exit, Course ENtry

        ROS_INFO("Goal Publisher initialized with %zu goals", goals_.size());
    }
    double distance(std::vector<double> a, std::vector<double> b)
    {
        return std::sqrt(std::pow(a[0] - b[0],2)+std::pow(a[1]-b[1],2));
    }
    
    void odomCallback(nav_msgs::Odometry odom)
    {
        // ROS_INFO("Odom Updated: new coords are: %f %f ",odom.pose.pose.position.x,odom.pose.pose.position.y);
        curr_pos[0] = odom.pose.pose.position.x;
        curr_pos[1] = odom.pose.pose.position.y;
    }
    bool check_cond()
    {
        if(this->distance(curr_pos,goals_[current_goal_index_]) < 1.0)
        {
            current_goal_index_++;
            return true;
        }
        // ROS_INFO("Distance to next goal %f",this->distance(curr_pos,goals_[current_goal_index_]));
        return false;
    }
    void publishNextGoal()
    {
        if (current_goal_index_ >= goals_.size())
        {
            ROS_INFO("All goals have been published.");
            return;
        }

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        goal.pose.position.x = goals_[current_goal_index_][0];
        goal.pose.position.y = goals_[current_goal_index_][1];
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = goals_[current_goal_index_][2];
        goal.pose.orientation.y = goals_[current_goal_index_][3];
        goal.pose.orientation.z = goals_[current_goal_index_][4];
        goal.pose.orientation.w = goals_[current_goal_index_][5];

 

        goal_pub_.publish(goal);

        ROS_INFO("Published goal %d: (%.2f, %.2f)", 
                 current_goal_index_ + 1, 
                 goals_[current_goal_index_][0], 
                 goals_[current_goal_index_][1]);

        // current_goal_index_++;
    }

    bool hasMoreGoals()
    {
        return current_goal_index_ < goals_.size();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher_node");
    GoalPublisher publisher;

    while (publisher.hasMoreGoals() && ros::ok())
    {
        if(publisher.check_cond())
            publisher.publishNextGoal();
        ros::spinOnce();
    }

    ROS_INFO("All goals have been published. Exiting.");
    return 0;
}
