#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

class ShortTermPlanner {
public:
    ShortTermPlanner() : current_target_index_(0), goal_sent_(false) {
        ros::NodeHandle nh;

        sub_pose_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &ShortTermPlanner::poseCallback, this);
        pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        if (!loadTargetPositions("/tmp/car_positions.txt")) {
            ROS_ERROR("Failed to load target positions. Shutting down.");
            ros::shutdown();
        } else {
            ROS_INFO("Loaded %lu target positions.", target_positions_.size());
            sendNextGoal();
        }
    }

private:
    ros::Subscriber sub_pose_;
    ros::Publisher pub_goal_;

    std::vector<std::pair<double, double>> target_positions_;
    size_t current_target_index_;
    bool goal_sent_;

    bool loadTargetPositions(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file: %s", filepath.c_str());
            return false;
        }

        std::string line;
        // 读取表头
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;

            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                target_positions_.emplace_back(x, y);
            }
        }

        file.close();
        return !target_positions_.empty();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (current_target_index_ >= target_positions_.size()) {
            ROS_INFO_THROTTLE(5, "All targets published. Waiting...");
            return;
        }

        double current_x = msg->pose.position.x;
        double current_y = msg->pose.position.y;

        double target_x = target_positions_[current_target_index_].first;
        double target_y = target_positions_[current_target_index_].second;

        double dist = std::hypot(current_x - target_x, current_y - target_y);

        if (dist < 2.5) {
            ROS_INFO("Reached target %lu: (%.2f, %.2f)", current_target_index_, target_x, target_y);
            current_target_index_++;
            if (current_target_index_ < target_positions_.size()) {
                sendNextGoal();
            } else {
                ROS_INFO("All target positions have been sent.");
            }
        }
    }

    void sendNextGoal() {
        if (current_target_index_ >= target_positions_.size())
            return;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "world";  // 根据实际情况，可能是 "map" 或其他
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = target_positions_[current_target_index_].first;
        goal.pose.position.y = target_positions_[current_target_index_].second;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;

        pub_goal_.publish(goal);
        ROS_INFO("Published goal %lu: (%.2f, %.2f)", current_target_index_, goal.pose.position.x, goal.pose.position.y);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "short_term_planner_node");
    ShortTermPlanner planner;
    ros::spin();
    return 0;
}
