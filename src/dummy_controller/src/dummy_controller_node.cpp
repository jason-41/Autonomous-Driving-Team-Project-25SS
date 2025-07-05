#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>


// 手动实现 clamp 函数
namespace my_utils {
    template<typename T>
    const T& clamp(const T& v, const T& lo, const T& hi) {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
}

class PathTrackingController {
public:
    PathTrackingController() : 
        tf_listener(tf_buffer),
        current_speed(0.0),
        target_speed(0.0),
        last_error(0.0),
        integral(0.0) {
        
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        
        // 参数加载
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
        
        // 订阅器和发布器
        path_sub = nh.subscribe("planned_trajectory", 1, &PathTrackingController::pathCallback, this);
        control_pub = nh.advertise<simulation::VehicleControl>("car_command", 1);
        
        // 定时器
        control_timer = nh.createTimer(ros::Duration(0.05), &PathTrackingController::controlLoop, this);
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (!msg->poses.empty()) {
            current_path = *msg;
            ROS_INFO("Received new path with %lu points", msg->poses.size());
        }
    }
    
    void controlLoop(const ros::TimerEvent&) {
        if (current_path.poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "No path received yet");
            return;
        }
        
        // 获取车辆当前位姿
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer.lookupTransform("world", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        
        geometry_msgs::PoseStamped current_pose;
        current_pose.header = transform.header;
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.position.z = transform.transform.translation.z;
        current_pose.pose.orientation = transform.transform.rotation;
        
        // 寻找路径上最近点
        size_t closest_idx = findClosestPoint(current_pose);
        
        // 计算前瞻点
        geometry_msgs::PoseStamped lookahead_point = getLookaheadPoint(current_pose, closest_idx);
        
        // 计算横向误差（交叉跟踪误差）
        double cross_track_error = calculateCrossTrackError(current_pose, lookahead_point);
        
        // 计算转向角（使用PID控制）
        double steering_angle = calculateSteeringAngle(cross_track_error);
        
        // 计算目标速度（基于路径曲率）
        target_speed = calculateTargetSpeed(closest_idx);
        
        // 计算油门/刹车指令（使用PID控制）
        std::pair<double, double> throttle_brake = calculateThrottleBrake();
        
        // 发布控制指令
        simulation::VehicleControl control_msg;
        control_msg.Throttle = throttle_brake.first;
        control_msg.Brake = throttle_brake.second;
        control_msg.Steering = steering_angle;
        control_msg.Reserved = 0.0;
        
        control_pub.publish(control_msg);
    }
    
private:
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
        // 动态调整前瞻距离基于当前速度
        double dynamic_lookahead = my_utils::clamp(current_speed * 0.5, min_lookahead, max_lookahead);
        
        double accumulated_dist = 0.0;
        geometry_msgs::PoseStamped lookahead_point = current_path.poses[closest_idx];
        
        for (size_t i = closest_idx; i < current_path.poses.size() - 1; ++i) {
            double dx = current_path.poses[i+1].pose.position.x - current_path.poses[i].pose.position.x;
            double dy = current_path.poses[i+1].pose.position.y - current_path.poses[i].pose.position.y;
            double segment_length = std::hypot(dx, dy);
            
            accumulated_dist += segment_length;
            
            if (accumulated_dist >= dynamic_lookahead) {
                double alpha = (accumulated_dist - dynamic_lookahead) / segment_length;
                lookahead_point.pose.position.x = current_path.poses[i].pose.position.x + alpha * dx;
                lookahead_point.pose.position.y = current_path.poses[i].pose.position.y + alpha * dy;
                break;
            }
        }
        
        return lookahead_point;
    }
    
    double calculateCrossTrackError(const geometry_msgs::PoseStamped& current_pose, 
                                   const geometry_msgs::PoseStamped& lookahead_point) {
        // 计算车辆到前瞻点的向量
        double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;
        
        // 计算车辆航向角
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 计算横向误差
        double path_angle = std::atan2(dy, dx);
        double angle_diff = path_angle - yaw;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff)); // 归一化到[-π, π]
        
        double cross_track_error = std::hypot(dx, dy) * std::sin(angle_diff);
        
        return cross_track_error;
    }
    
    double calculateSteeringAngle(double cross_track_error) {
        // PID控制计算转向角
        double error_derivative = (cross_track_error - last_error) / 0.05;
        integral += cross_track_error * 0.05;
        
        // 限制积分项防止积分饱和
        integral = my_utils::clamp(integral, -1.0, 1.0);
        
        double steering_angle = kp_steer * cross_track_error + 
                              ki_steer * integral + 
                              kd_steer * error_derivative;
        
        last_error = cross_track_error;
        
        // 限制转向角在[-1, 1]范围内
        return my_utils::clamp(steering_angle, -1.0, 1.0);
    }
    
    double calculateTargetSpeed(size_t closest_idx) {
        // 简单实现：根据路径曲率调整速度
        if (closest_idx + 5 >= current_path.poses.size()) {
            return max_speed * 0.3; // 接近终点时减速
        }
        
        // 计算路径曲率（通过前后点角度变化）
        double angle1 = std::atan2(
            current_path.poses[closest_idx+1].pose.position.y - current_path.poses[closest_idx].pose.position.y,
            current_path.poses[closest_idx+1].pose.position.x - current_path.poses[closest_idx].pose.position.x);
        
        double angle2 = std::atan2(
            current_path.poses[closest_idx+5].pose.position.y - current_path.poses[closest_idx+4].pose.position.y,
            current_path.poses[closest_idx+5].pose.position.x - current_path.poses[closest_idx+4].pose.position.x);
        
        double angle_diff = std::abs(std::atan2(std::sin(angle2-angle1), std::cos(angle2-angle1)));
        
        // 曲率越大，速度越小
        double curvature_factor = 1.0 - std::min(angle_diff / (M_PI/4), 0.8);
        return max_speed * curvature_factor;
    }
    
    std::pair<double, double> calculateThrottleBrake() {
        // 简单PID速度控制
        double speed_error = target_speed - current_speed;
        double throttle = kp_speed * speed_error;
        
        // 如果需要减速，使用刹车
        double brake = 0.0;
        if (speed_error < -0.5) {
            brake = std::min(std::abs(speed_error) * 0.5, 1.0);
            throttle = 0.0;
        }
        
        // 限制输出范围
        throttle = my_utils::clamp(throttle, 0.0, 1.0);
        brake = my_utils::clamp(brake, 0.0, 1.0);
        
        return {throttle, brake};
    }
    
    // ROS相关
    ros::Subscriber path_sub;
    ros::Publisher control_pub;
    ros::Timer control_timer;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    
    // 路径数据
    nav_msgs::Path current_path;
    
    // 控制参数
    double max_speed;
    double kp_steer, ki_steer, kd_steer;
    double kp_speed, ki_speed, kd_speed;
    double lookahead_distance;
    double min_lookahead, max_lookahead;
    double brake_distance;
    
    // 状态变量
    double current_speed;
    double target_speed;
    double last_error;
    double integral;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracking_controller");
    PathTrackingController controller;
    ros::spin();
    return 0;
}





/*
#include <ros/ros.h>
#include "simulation/VehicleControl.h"
//#include <simulation/VehicleControl.h>
#include "math.h"
constexpr float loop_interval = 0.05;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_controller_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<simulation::VehicleControl>("car_command", 1);
    ros::Rate loop_rate(1 / loop_interval);
    float elapsed_time = 0.0f;

    while (ros::ok())
    {
        simulation::VehicleControl msg;
        msg.Throttle = 0.5f; // Throttle value from -1 to 1, this is the torque applied to the motors
        msg.Steering =  sin(6.28 * elapsed_time) * 0.5; //Steering value from -1 to 1, in which: positive value <=> turning right
        msg.Brake = 0.0f; // Brake value from 0 to 1, this will apply brake torque to stop the car
        msg.Reserved = 0.0f; // Not used!

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        elapsed_time += loop_interval;

    }

    return 0;
}

*/
