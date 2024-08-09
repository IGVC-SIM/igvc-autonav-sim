#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <algorithm>

class FilteredMapNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid filtered_map_;
    std::vector<int8_t> filtered_array_;
    std::vector<int> update_count_;
    bool is_initialized_;
    int max_updates_;

public:
    FilteredMapNode() : is_initialized_(false) {
        nh_.param("max_updates", max_updates_, 1); // Default to 1 if not specified
        ROS_INFO("Max updates per cell: %d", max_updates_);

        map_sub_ = nh_.subscribe("/projected_map", 1, &FilteredMapNode::mapCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/filtered_map", 1);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (!is_initialized_) {
            filtered_map_ = *msg;
            filtered_array_.resize(msg->info.width * msg->info.height, -1);
            update_count_.resize(msg->info.width * msg->info.height, 0);
            is_initialized_ = true;
        }

        int x_offset = static_cast<int>((msg->info.origin.position.x - filtered_map_.info.origin.position.x) / msg->info.resolution);
        int y_offset = static_cast<int>((msg->info.origin.position.y - filtered_map_.info.origin.position.y) / msg->info.resolution);

        int new_width = std::max(static_cast<int>(filtered_map_.info.width), x_offset + static_cast<int>(msg->info.width));
        int new_height = std::max(static_cast<int>(filtered_map_.info.height), y_offset + static_cast<int>(msg->info.height));

        std::vector<int8_t> new_filtered_array(new_width * new_height, -1);
        std::vector<int> new_update_count(new_width * new_height, 0);

        // Copy existing data to the new array
        for (int y = 0; y < filtered_map_.info.height; ++y) {
            for (int x = 0; x < filtered_map_.info.width; ++x) {
                int new_y = y + std::max(0, -y_offset);
                int new_x = x + std::max(0, -x_offset);
                if (new_y < new_height && new_x < new_width) {
                    int old_index = y * filtered_map_.info.width + x;
                    int new_index = new_y * new_width + new_x;
                    new_filtered_array[new_index] = filtered_array_[old_index];
                    new_update_count[new_index] = update_count_[old_index];
                }
            }
        }

        // Update with new known cells
        for (int y = 0; y < msg->info.height; ++y) {
            for (int x = 0; x < msg->info.width; ++x) {
                int new_y = y_offset + y;
                int new_x = x_offset + x;
                if (new_y >= 0 && new_y < new_height && new_x >= 0 && new_x < new_width) {
                    int new_index = new_y * new_width + new_x;
                    int8_t incoming_value = msg->data[y * msg->info.width + x];
                    if (incoming_value != -1 && new_update_count[new_index] < max_updates_) {
                        new_filtered_array[new_index] = incoming_value;
                        new_update_count[new_index]++;
                    }
                }
            }
        }

        // Update filtered map metadata
        filtered_map_.info.width = new_width;
        filtered_map_.info.height = new_height;
        filtered_map_.info.origin.position.x = std::min(filtered_map_.info.origin.position.x, msg->info.origin.position.x);
        filtered_map_.info.origin.position.y = std::min(filtered_map_.info.origin.position.y, msg->info.origin.position.y);

        // Update filtered map data
        filtered_map_.data = new_filtered_array;
        filtered_array_ = std::move(new_filtered_array);
        update_count_ = std::move(new_update_count);

        // Publish the updated filtered map
        map_pub_.publish(filtered_map_);
    }

    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "filtered_map_node");
    FilteredMapNode node;
    node.run();
    return 0;
}