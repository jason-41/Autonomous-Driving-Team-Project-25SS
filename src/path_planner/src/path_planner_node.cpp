#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y;
    float cost, heuristic;
    Node* parent;

    Node(int x_, int y_, float cost_, float heuristic_, Node* parent_ = nullptr)
        : x(x_), y(y_), cost(cost_), heuristic(heuristic_), parent(parent_) {}

    float totalCost() const { return cost + heuristic; }

    bool operator<(const Node& other) const {
        return totalCost() > other.totalCost();  // min-heap
    }
};

class PathPlanner {
public:
    PathPlanner() : map_received_(false), car_pose_received_(false), goal_received_(false), goal_reached_(false) {
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
        inflateMap(3);
        map_received_ = true;
        ROS_INFO("Map received and inflated: %d x %d", width_, height_);
    }

    void inflateMap(int inflation_radius) {
        std::vector<int8_t> inflated_data = map_.data;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                int idx = toIndex(x, y);
                if (map_.data[idx] >= 80) {
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (inBounds(nx, ny)) {
                                int n_idx = toIndex(nx, ny);
                                if (inflated_data[n_idx] < 80)
                                    inflated_data[n_idx] = 79;
                            }
                        }
                    }
                }
            }
        }
        map_.data = inflated_data;
    }

    void carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        car_x_ = (msg->pose.position.x - origin_x_) / resolution_;
        car_y_ = (msg->pose.position.y - origin_y_) / resolution_;
        car_pose_received_ = true;

        // 检查是否已接近目标点，如果是，则跳过路径规划
        if (goal_received_) {
            double dx = car_x_ - goal_x_;
            double dy = car_y_ - goal_y_;
            double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
            if (dist < 4.0) {
                if (!goal_reached_) {
                    ROS_INFO("Car is within %.2f meters of goal, skipping further path planning.", dist);
                    goal_reached_ = true;
                }
                return;
            }
        }

        // 若满足条件则执行路径规划
        if (map_received_ && goal_received_ && !goal_reached_)
            computePath();
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_x_ = (msg->pose.position.x - origin_x_) / resolution_;
        goal_y_ = (msg->pose.position.y - origin_y_) / resolution_;
        goal_received_ = true;
        goal_reached_ = false;  // 接收到新目标，重置标志
        ROS_INFO("Goal grid received: (%d, %d)", goal_x_, goal_y_);

        if (map_received_ && car_pose_received_)
            computePath();
        else
            ROS_WARN("Waiting for map and car pose...");
    }

    void computePath() {
        if (!inBounds(car_x_, car_y_)) {
            ROS_WARN("Start grid (%d, %d) is out of bounds. Searching for nearby valid start...", car_x_, car_y_);
            bool found = false;
            for (int r = 1; r <= 5 && !found; ++r) {
                for (int dx = -r; dx <= r; ++dx) {
                   for (int dy = -r; dy <= r; ++dy) {
                        int nx = car_x_ + dx;
                        int ny = car_y_ + dy;
                        if (inBounds(nx, ny) && !isOccupied(nx, ny)) {
                            car_x_ = nx;
                            car_y_ = ny;
                            found = true;
                            ROS_INFO("Relocated out-of-bounds start to (%d, %d)", car_x_, car_y_);
                            break;
                        }
                    }
                   if (found) break;
                }
            }
            if (!found) {
                ROS_ERROR("Failed to find valid start near out-of-bounds position.");
                return;
            }
        }

        if (!inBounds(goal_x_, goal_y_)) {
            ROS_ERROR("Goal grid out of bounds: (%d, %d)", goal_x_, goal_y_);
            return;
        }

        if (isOccupied(car_x_, car_y_)) {
            ROS_WARN("Car appears in obstacle. Trying to relocate start point.");
            bool found = false;
            for (int dx = -1; dx <= 1 && !found; ++dx) {
                for (int dy = -1; dy <= 1 && !found; ++dy) {
                    int nx = car_x_ + dx;
                    int ny = car_y_ + dy;
                    if (inBounds(nx, ny) && !isOccupied(nx, ny)) {
                        car_x_ = nx;
                        car_y_ = ny;
                        found = true;
                        ROS_INFO("Relocated start to (%d, %d)", car_x_, car_y_);
                    }
                }
            }
            if (!found) {
                ROS_ERROR("No valid start found near current car position.");
                return;
            }
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
                goal_node = new Node(current);
                ROS_INFO("Goal reached!");
                break;
            }

            for (const auto& dir : directions_) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];

                if (!inBounds(nx, ny) || isOccupied(nx, ny) || closed_list[toIndex(nx, ny)])
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
            while (n) {
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
            ROS_WARN("Path not found.");
        }
    }

private:
    ros::Subscriber map_sub_;
    ros::Publisher path_pub_;
    ros::Subscriber car_pose_sub_;
    ros::Subscriber goal_sub_;
    nav_msgs::OccupancyGrid map_;
    int width_, height_;
    double resolution_, origin_x_, origin_y_;
    int car_x_, car_y_, goal_x_, goal_y_;
    bool map_received_, car_pose_received_, goal_received_;
    bool goal_reached_;  // 🆕 标志：是否已经到达目标

    const std::vector<std::vector<int>> directions_ = {
        {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {-1,-1}, {1,-1}, {-1,1}
    };

    int toIndex(int x, int y) const {
        return y * width_ + x;
    }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    bool isOccupied(int x, int y) const {
        int idx = toIndex(x, y);
        if (idx < 0 || idx >= map_.data.size())
            return true;
        int val = map_.data[idx];
        return val >= 79;
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
