#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <cmath>

struct Node {
    int x, y;
    float cost, heuristic;
    Node* parent;

    Node(int x_, int y_, float cost_, float heuristic_, Node* parent_ = nullptr)
        : x(x_), y(y_), cost(cost_), heuristic(heuristic_), parent(parent_) {}

    float totalCost() const { return cost + heuristic; }

    bool operator<(const Node& other) const {
        return totalCost() > other.totalCost();  // 小顶堆
    }
};

class PathPlanner {
public:
    PathPlanner() : map_received_(false), car_pose_received_(false), goal_received_(false) {
        ros::NodeHandle nh;
        map_sub_ = nh.subscribe("/projected_map", 1, &PathPlanner::mapCallback, this);
        path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1, true);
        car_pose_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &PathPlanner::carPoseCallback, this);
        goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &PathPlanner::goalCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        width_ = map_.info.width;
        height_ = map_.info.height;
        resolution_ = map_.info.resolution;
        origin_x_ = map_.info.origin.position.x;
        origin_y_ = map_.info.origin.position.y;
        map_received_ = true;

        ROS_INFO("Map received: %d x %d", width_, height_);
    }

    void carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        car_x_ = (msg->pose.position.x - origin_x_) / resolution_;
        car_y_ = (msg->pose.position.y - origin_y_) / resolution_;
        car_pose_received_ = true;

        ROS_INFO("Current car grid: (%d, %d)", car_x_, car_y_);
        
        if (map_received_ && car_pose_received_ && goal_received_) {
            computePath();
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_x_ = (msg->pose.position.x - origin_x_) / resolution_;
        goal_y_ = (msg->pose.position.y - origin_y_) / resolution_;
        goal_received_ = true;

        ROS_INFO("Goal grid received: (%d, %d)", goal_x_, goal_y_);

        if (map_received_ && car_pose_received_) {
            computePath();
        } else {
            ROS_WARN("Cannot compute path yet, waiting for map and car position.");
        }
    }

    void computePath() {
        if (car_x_ < 0 || car_x_ >= width_ || car_y_ < 0 || car_y_ >= height_) {
            ROS_ERROR("Start grid out of bounds: (%d, %d)", car_x_, car_y_);
            return;
        }
        if (goal_x_ < 0 || goal_x_ >= width_ || goal_y_ < 0 || goal_y_ >= height_) {
            ROS_ERROR("Goal grid out of bounds: (%d, %d)", goal_x_, goal_y_);
            return;
        }

        std::priority_queue<Node> open_list;
        std::vector<bool> closed_list(width_ * height_, false);

        Node* start_node = new Node(car_x_, car_y_, 0, heuristic(car_x_, car_y_, goal_x_, goal_y_));
        open_list.push(*start_node);

        Node* goal_node = nullptr;

        while (!open_list.empty()) {
            Node current = open_list.top();
            open_list.pop();

            int idx = toIndex(current.x, current.y);
            if (closed_list[idx]) continue;
            closed_list[idx] = true;

            if (current.x == goal_x_ && current.y == goal_y_) {
                ROS_INFO("Goal reached!");
                goal_node = new Node(current);
                break;
            }

            for (const auto& dir : directions_) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];

                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
                    continue;

                if (isOccupied(nx, ny))
                    continue;

                if (closed_list[toIndex(nx, ny)])
                    continue;

                float new_cost = current.cost + distance(current.x, current.y, nx, ny);
                float h = heuristic(nx, ny, goal_x_, goal_y_);
                Node* neighbor = new Node(nx, ny, new_cost, h, new Node(current));
                open_list.push(*neighbor);
            }
        }

        if (goal_node) {
            nav_msgs::Path path;
            path.header.frame_id = map_.header.frame_id;
            path.header.stamp = ros::Time::now();

            Node* n = goal_node;
            while (n != nullptr) {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = origin_x_ + (n->x + 0.5) * resolution_;
                pose.pose.position.y = origin_y_ + (n->y + 0.5) * resolution_;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
                n = n->parent;
            }

            std::reverse(path.poses.begin(), path.poses.end());
            path_pub_.publish(path);
            ROS_INFO("Path published with %lu points", path.poses.size());
        } else {
            ROS_WARN("Path not found");
        }
    }

private:
    ros::Subscriber map_sub_;
    ros::Publisher path_pub_;
    ros::Subscriber car_pose_sub_;
    ros::Subscriber goal_sub_;
    nav_msgs::OccupancyGrid map_;
    int width_, height_;
    double resolution_;
    double origin_x_, origin_y_;
    int car_x_, car_y_;
    int goal_x_, goal_y_;

    bool map_received_ = false;
    bool car_pose_received_ = false;
    bool goal_received_ = false;

    const std::vector<std::vector<int>> directions_ = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {-1,-1}, {1,-1}, {-1,1}
    };

    int toIndex(int x, int y) const {
        return y * width_ + x;
    }

    bool isOccupied(int x, int y) const {
        int idx = toIndex(x, y);
        int val = map_.data[idx];
        return (val > 50 || val < 0);
    }

    float heuristic(int x, int y, int gx, int gy) const {
        return std::hypot(gx - x, gy - y);
    }

    float distance(int x1, int y1, int x2, int y2) const {
        return std::hypot(x2 - x1, y2 - y1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    PathPlanner planner;
    ros::spin();
    return 0;
}