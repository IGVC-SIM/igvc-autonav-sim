#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>


std::vector<std::vector<double>> updateGoalsWithIntermediates(const std::vector<std::vector<double>>& original_goals, double max_distance = 4.0) {
    std::vector<std::vector<double>> updated_goals;
    
    for (size_t i = 0; i < original_goals.size() - 1; ++i) {
        const auto& start = original_goals[i];
        const auto& end = original_goals[i + 1];
        
        updated_goals.push_back(start);
        
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance > max_distance) {
            int num_intermediates = std::ceil(distance / max_distance) - 1;
            double step_x = dx / (num_intermediates + 1);
            double step_y = dy / (num_intermediates + 1);
            
            for (int j = 1; j <= num_intermediates; ++j) {
                std::vector<double> intermediate_goal = start;
                intermediate_goal[0] += j * step_x;
                intermediate_goal[1] += j * step_y;
                
                // Linearly interpolate orientation
                for (int k = 2; k < 6; ++k) {
                    intermediate_goal[k] = start[k] + (end[k] - start[k]) * (j / (num_intermediates + 1.0));
                }
                
                updated_goals.push_back(intermediate_goal);
            }
        }
    }
    
    updated_goals.push_back(original_goals.back());
    
    return updated_goals;
}


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
        
        // Initialize original goals
        goals_ = {{10.981856913385355, 20.638044400288646, 0.0025062586038971544, 0.0011526581582510702, -0.9204649425761795, 0.3908154034006479},
{5.361658605594936, 10.8221819978611, 0.002450488413233407, -3.1195735068890354e-05, -0.9998266730773296, -0.018455837463561522},
{-7.931561996215021, 11.314941002522998, 0.0026270809339856184, -7.782901614172671e-05, -0.9998286404840551, -0.018324356911410388},
{-14.127625860234343, 22.478713612828365, 0.002357779524645117, 5.732174590407077e-06, -0.9998937460297591, -0.0143853221525018}};
        
        // Update goals with intermediates
        goals_ = updateGoalsWithIntermediates(goals_);
        
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
        double distance = this->distance(curr_pos,goals_[current_goal_index_]);
        if(distance < 1.0 || (current_goal_index_==0 && distance < 7.0))
        {
            current_goal_index_++;
            return true;
        }
        ROS_INFO("Distance to next goal %f",distance);
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
    int i = 0;
    while (publisher.hasMoreGoals() && ros::ok())
    {
        if(publisher.check_cond())
            publisher.publishNextGoal();
        ros::spinOnce();
    }

    ROS_INFO("All goals have been published. Exiting.");
    return 0;
}
