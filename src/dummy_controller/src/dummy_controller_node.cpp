#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <string>
#include <iomanip>
#include <sys/stat.h>
// *** Added ***
#include <std_msgs/String.h>
#include <cmath>
#include <algorithm>

namespace my_utils {
    template<typename T>
    const T& clamp(const T& v, const T& lo, const T& hi) {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
}

class PathTrackingController {
private:
    // TF components
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Subscribers and publishers
    ros::Subscriber path_sub;
    ros::Subscriber vehicle_state_sub;
    ros::Subscriber traffic_sub;          // *** Added ***
    ros::Publisher control_pub;
    ros::Timer control_timer;

    // Path and vehicle state
    nav_msgs::Path current_path;
    geometry_msgs::Pose current_vehicle_pose;
    bool has_vehicle_state;

    // Control parameters
    double max_speed;
    double kp_steer, ki_steer, kd_steer;
    double kp_speed, ki_speed, kd_speed;
    double lookahead_distance;
    double min_lookahead, max_lookahead;
    double brake_distance;

    // Internal state
    double current_speed;
    double target_speed;
    double last_error;
    double integral;
    std::string traffic_light_state;      // *** Added ***

    // Output file for logging
    std::string output_file;

public:
    PathTrackingController() :
        tf_listener(tf_buffer),
        has_vehicle_state(false),
        current_speed(0.0),
        target_speed(0.0),
        last_error(0.0),
        integral(0.0),
        traffic_light_state("UNKNOWN"),    // *** Added ***
        output_file("/home/kingsbob/Desktop/Intro2ROS/project/Kong/src/dummy_controller/src/vehicle_state.txt")
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Load parameters
        private_nh.param("max_speed", max_speed, 5.0);
        private_nh.param("kp_steer", kp_steer, 0.5);
        private_nh.param("ki_steer", ki_steer, 0.01);
        private_nh.param("kd_steer", kd_steer, 0.1);
        private_nh.param("kp_speed", kp_speed, 0.3);
        private_nh.param("ki_speed", ki_speed, 0.01);
        private_nh.param("kd_speed", kd_speed, 0.05);
        private_nh.param("lookahead_distance", lookahead_distance, 3.0);
        private_nh.param("min_lookahead", min_lookahead, 1.5);
        private_nh.param("max_lookahead", max_lookahead, 5.0);
        private_nh.param("brake_distance", brake_distance, 2.0);

        // Setup subscribers/publishers
        path_sub = nh.subscribe("planned_trajectory", 1, &PathTrackingController::pathCallback, this);
        vehicle_state_sub = nh.subscribe("vehicle_state", 1, &PathTrackingController::vehicleStateCallback, this);
        traffic_sub = nh.subscribe("/traffic_state", 10, &PathTrackingController::trafficLightCallback, this);  // *** Added ***
        control_pub = nh.advertise<simulation::VehicleControl>("car_command", 1);
        control_timer = nh.createTimer(ros::Duration(0.05), &PathTrackingController::controlLoop, this);

        initializeOutputFile();
    }

private:
    // Traffic light state callback
    void trafficLightCallback(const std_msgs::String::ConstPtr& msg) {  // *** Added ***
        traffic_light_state = msg->data;
    }

    // Original callbacks
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (!msg->poses.empty()) {
            current_path = *msg;
            ROS_INFO("Received new path with %lu points", msg->poses.size());
        }
    }

    void vehicleStateCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        current_vehicle_pose = *msg;
        has_vehicle_state = true;
        writePoseToFile(current_vehicle_pose);
    }

    void controlLoop(const ros::TimerEvent&) {
        if (current_path.poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "No path received yet");
            return;
        }

        geometry_msgs::PoseStamped current_pose;
        if (has_vehicle_state) {
            current_pose.header.stamp = ros::Time::now();
            current_pose.header.frame_id = "world";
            current_pose.pose = current_vehicle_pose;
        } else {
            try {
                auto transform = tf_buffer.lookupTransform("world", "base_link", ros::Time(0));
                current_pose.header = transform.header;
                //current_pose.pose.position = transform.transform.translation;
                current_pose.pose.position.x = transform.transform.translation.x;
                current_pose.pose.position.y = transform.transform.translation.y;
                current_pose.pose.position.z = transform.transform.translation.z;

                current_pose.pose.orientation = transform.transform.rotation;
                writePoseToFile(current_pose.pose);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                return;
            }
        }

        size_t closest_idx = findClosestPoint(current_pose);
        auto lookahead_point = getLookaheadPoint(current_pose, closest_idx);
        double cte = calculateCrossTrackError(current_pose, lookahead_point);
        double steering_angle = calculateSteeringAngle(cte);

        target_speed = calculateTargetSpeed(closest_idx);  // contains added logic

        auto tb = calculateThrottleBrake();
        simulation::VehicleControl control_msg;
        control_msg.Throttle = tb.first;
        control_msg.Brake = tb.second;
        control_msg.Steering = steering_angle;
        control_msg.Reserved = 0.0;
        control_pub.publish(control_msg);
    }

    // Original helper functions kept unchanged:
    size_t findClosestPoint(const geometry_msgs::PoseStamped& current_pose) {
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < current_path.poses.size(); ++i) {
            double dx = current_path.poses[i].pose.position.x - current_pose.pose.position.x;
            double dy = current_path.poses[i].pose.position.y - current_pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    geometry_msgs::PoseStamped getLookaheadPoint(const geometry_msgs::PoseStamped& current_pose, size_t closest_idx) {
        double dynamic_lookahead = my_utils::clamp(current_speed * 0.5, min_lookahead, max_lookahead);
        double accumulated_dist = 0.0;
        geometry_msgs::PoseStamped point = current_path.poses[closest_idx];
        for (size_t i = closest_idx; i < current_path.poses.size() - 1; ++i) {
            double dx = current_path.poses[i+1].pose.position.x - current_path.poses[i].pose.position.x;
            double dy = current_path.poses[i+1].pose.position.y - current_path.poses[i].pose.position.y;
            accumulated_dist += std::hypot(dx, dy);
            if (accumulated_dist >= dynamic_lookahead) {
                double alpha = (accumulated_dist - dynamic_lookahead) / std::hypot(dx, dy);
                point.pose.position.x = current_path.poses[i].pose.position.x + alpha * dx;
                point.pose.position.y = current_path.poses[i].pose.position.y + alpha * dy;
                break;
            }
        }
        return point;
    }

    double calculateCrossTrackError(const geometry_msgs::PoseStamped& current_pose, 
                                    const geometry_msgs::PoseStamped& lookahead_point) {
        double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;

        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double path_angle = std::atan2(dy, dx);
        double angle_diff = std::atan2(std::sin(path_angle - yaw), std::cos(path_angle - yaw));
        return std::hypot(dx, dy) * std::sin(angle_diff);
    }

    double calculateSteeringAngle(double cte) {
        double error_derivative = (cte - last_error) / 0.05;
        integral += cte * 0.05;
        integral = my_utils::clamp(integral, -1.0, 1.0);
        double angle = kp_steer * cte + ki_steer * integral + kd_steer * error_derivative;
        last_error = cte;
        return my_utils::clamp(angle, -1.0, 1.0);
    }

    double calculateTargetSpeed(size_t closest_idx) {
        // *** Added: stop on red light ***
        if (traffic_light_state == "RED") {
            ROS_WARN_THROTTLE(1.0, "Red light detected! Stop!");
            return 0.0;
        }
        // Original curvature-based speed logic
        if (closest_idx + 5 >= current_path.poses.size()) {
            return max_speed * 0.3;
        }
        double angle1 = std::atan2(
            current_path.poses[closest_idx+1].pose.position.y - current_path.poses[closest_idx].pose.position.y,
            current_path.poses[closest_idx+1].pose.position.x - current_path.poses[closest_idx].pose.position.x);
        double angle2 = std::atan2(
            current_path.poses[closest_idx+5].pose.position.y - current_path.poses[closest_idx+4].pose.position.y,
            current_path.poses[closest_idx+5].pose.position.x - current_path.poses[closest_idx+4].pose.position.x);
        double angle_diff = std::abs(std::atan2(std::sin(angle2 - angle1), std::cos(angle2 - angle1)));
        double curvature_factor = 1.0 - std::min(angle_diff / (M_PI/4), 0.8);
        return max_speed * curvature_factor;
    }

    std::pair<double, double> calculateThrottleBrake() {
        double speed_error = target_speed - current_speed;
        double throttle = kp_speed * speed_error;
        double brake = 0.0;
        if (speed_error < -0.5) {
            brake = std::min(std::abs(speed_error) * 0.5, 1.0);
            throttle = 0.0;
        }
        return { my_utils::clamp(throttle, 0.0, 1.0), my_utils::clamp(brake, 0.0, 1.0) };
    }

    // File output routines from original source
    bool checkFileWritable() {
        std::ofstream testfile(output_file, std::ios::app);
        if (!testfile) {
            ROS_ERROR("文件创建失败: %s (错误: %s)", output_file.c_str(), strerror(errno));
            return false;
        }
        return true;
    }

    void initializeOutputFile() {
        std::ofstream outfile(output_file, std::ios::out | std::ios::trunc);
        if (outfile) {
            outfile << "timestamp,x,y,z,qx,qy,qz,qw\n";
        } else {
            ROS_FATAL("无法初始化文件: %s (错误: %s)", output_file.c_str(), strerror(errno));
            exit(1);
        }
    }

    void writePoseToFile(const geometry_msgs::Pose& pose) {
        std::ofstream outfile(output_file, std::ios::out | std::ios::app);
        if (outfile) {
            outfile << std::fixed << std::setprecision(6)
                    << ros::Time::now().toSec() << ","
                    << pose.position.x << ","
                    << pose.position.y << ","
                    << pose.position.z << ","
                    << pose.orientation.x << ","
                    << pose.orientation.y << ","
                    << pose.orientation.z << ","
                    << pose.orientation.w << "\n";
        } else {
            ROS_ERROR_THROTTLE(1.0, "写入失败: %s (错误: %s)", output_file.c_str(), strerror(errno));
        }
    }

public:
    // Public interface unchanged
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracking_controller");
    PathTrackingController controller;
    ros::spin();
    return 0;
}

