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

    // 比较器，用于优先队列
    bool operator<(const Node& other) const {
        return totalCost() > other.totalCost();  // 小顶堆
    }
};

class PathPlanner {
public:
    PathPlanner() {
        ros::NodeHandle nh;
        map_sub_ = nh.subscribe("/projected_map", 1, &PathPlanner::mapCallback, this);
        path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1, true);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        width_ = map_.info.width;
        height_ = map_.info.height;
        resolution_ = map_.info.resolution;
        origin_x_ = map_.info.origin.position.x;
        origin_y_ = map_.info.origin.position.y;

        ROS_INFO("Map received: %d x %d", width_, height_);

        // 暂时给定起点终点（可后续改为参数或订阅）
        int start_x = (int)((-62.80 - origin_x_) / resolution_);
        int start_y = (int)((-10 - origin_y_) / resolution_);
        // int goal_x = width_ - 10, goal_y = height_ - 10;
        int goal_x = (int)((-61.9 - origin_x_) / resolution_);
        int goal_y = (int)((8 - origin_y_) / resolution_);
        
        ROS_INFO("Start grid: (%d, %d)", start_x, start_y);
        ROS_INFO("Goal01 grid: (%d, %d)", goal_x, goal_y);

        if (start_x < 0 || start_x >= width_ || start_y < 0 || start_y >= height_) {
            ROS_ERROR("Start grid out of bounds: (%d, %d)", start_x, start_y);
            return;
        }
        if (goal_x < 0 || goal_x >= width_ || goal_y < 0 || goal_y >= height_) {
            ROS_ERROR("Goal grid out of bounds: (%d, %d)", goal_x, goal_y);
            return;
        }

        computePath(start_x, start_y, goal_x, goal_y);
    }

    void computePath(int start_x, int start_y, int goal_x, int goal_y) {
        std::priority_queue<Node> open_list;
        std::vector<bool> closed_list(width_ * height_, false);

        Node* start_node = new Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push(*start_node);

        Node* goal_node = nullptr;

        while (!open_list.empty()) {
            Node current = open_list.top();
            open_list.pop();

            int idx = toIndex(current.x, current.y);
            if (closed_list[idx]) continue;
            closed_list[idx] = true;

            if (current.x == goal_x && current.y == goal_y) {
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
                float h = heuristic(nx, ny, goal_x, goal_y);
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
    nav_msgs::OccupancyGrid map_;
    int width_, height_;
    double resolution_;
    double origin_x_, origin_y_;

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