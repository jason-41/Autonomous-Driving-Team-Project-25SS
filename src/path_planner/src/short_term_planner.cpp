#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <ros/package.h>
#include <cmath>

class ShortTermPlanner {
public:
    ShortTermPlanner()
        : current_target_index_(0), goal_sent_(false), pose_received_(false) {
        ros::NodeHandle nh;
        sub_pose_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &ShortTermPlanner::poseCallback, this);

        // 使用 latched publisher，确保监听者随时都能接收到
        pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

        std::string pkg_path = ros::package::getPath("path_planner");
        const std::string filepath = pkg_path + "/car_positions.txt";

        if (!loadTargetPositions(filepath)) {
            ROS_ERROR("Failed to load target positions from %s", filepath.c_str());
            ros::shutdown();
        } else {
            ROS_INFO("Loaded %lu target positions.", target_positions_.size());
        }
    }

private:
    ros::Subscriber sub_pose_;
    ros::Publisher pub_goal_;
    std::vector<std::pair<double, double>> target_positions_;  
    size_t current_target_index_;
    bool goal_sent_;
    bool pose_received_;

    bool loadTargetPositions(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file: %s", filepath.c_str());
            return false;
        }

        std::string line;
        if (!std::getline(file, line)) {
            ROS_ERROR("Target file %s is empty", filepath.c_str());
            file.close();
            return false;
        }

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            bool has_alpha = false;
            for (char c : line) {
                if (std::isalpha(c)) {
                    has_alpha = true;
                    break;
                }
            }
            if (has_alpha) continue;

            std::stringstream ss(line);
            std::string x_str, y_str;
            if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str)) {
                ROS_WARN("Malformed line: '%s', skipping", line.c_str());
                continue;
            }

            try {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                target_positions_.emplace_back(x, y);
            } catch (const std::exception&) {
                ROS_WARN("Invalid numeric in line: '%s', skipping", line.c_str());
                continue;
            }
        }

        file.close();
        return !target_positions_.empty();
    }

    void sendNextGoal() {
        if (current_target_index_ >= target_positions_.size()) {
            ROS_INFO_THROTTLE(5, "All targets published. Waiting...");
            return;
        }

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "world";  // 请确保和地图 frame 一致，如 "map"
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = target_positions_[current_target_index_].first;
        goal.pose.position.y = target_positions_[current_target_index_].second;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;

        pub_goal_.publish(goal);
        ROS_INFO("Published goal %lu: (%.2f, %.2f)",
                 current_target_index_, goal.pose.position.x, goal.pose.position.y);
        goal_sent_ = true;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double cx = msg->pose.position.x;
        double cy = msg->pose.position.y;

        if (!pose_received_) {
            pose_received_ = true;
            sendNextGoal();  // 初次收到位姿才发目标点
            return;
        }

        if (!goal_sent_ || current_target_index_ >= target_positions_.size())
            return;

        double tx = target_positions_[current_target_index_].first;
        double ty = target_positions_[current_target_index_].second;
        double dist = std::hypot(cx - tx, cy - ty);

        if (dist < 4.0) {
            ROS_INFO("Reached target %lu: (%.2f, %.2f)", current_target_index_, tx, ty);
            current_target_index_++;
            goal_sent_ = false;
            sendNextGoal();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "short_term_planner_node");
    ShortTermPlanner planner;
    ros::spin();
    return 0;
}
