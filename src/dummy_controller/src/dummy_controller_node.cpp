#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <simulation/VehicleControl.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <cmath>

geometry_msgs::PoseStamped current_target_pose;
geometry_msgs::TwistStamped current_velocity;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::TwistStamped current_twist;
std::string traffic_light_state = "GREEN";

bool has_target = false;
bool has_velocity = false;
bool has_pose = false;
bool has_twist = false;

constexpr float loop_interval = 0.05; // 20Hz 控制频率

// PID 控制参数
float Kp_lin = 2.0, Ki_lin = 0.0, Kd_lin = 0.2;
float Kp_ang = 4.0, Ki_ang = 0.0, Kd_ang = 0.4;

float integral_lin = 0.0, prev_error_lin = 0.0;
float integral_ang = 0.0, prev_error_ang = 0.0;

// 回调函数
void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_target_pose = *msg;
    has_target = true;
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_velocity = *msg;
    has_velocity = true;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    has_pose = true;
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_twist = *msg;
    has_twist = true;
}

void trafficLightCallback(const std_msgs::String::ConstPtr& msg) {
    traffic_light_state = msg->data;
}

// 获取当前朝向（从四元数转yaw）
float getYaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_controller_node");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<simulation::VehicleControl>("car_command", 1);
    ros::Subscriber target_pose_sub = nh.subscribe("/target_pose", 1, targetPoseCallback);
    ros::Subscriber velocity_sub    = nh.subscribe("/velocity", 1, velocityCallback);
    ros::Subscriber pose_sub        = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, poseCallback);
    ros::Subscriber twist_sub       = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 1, twistCallback);
    ros::Subscriber light_sub       = nh.subscribe("/traffic_light", 1, trafficLightCallback);

    ros::Rate loop_rate(1 / loop_interval);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!(has_target && has_velocity && has_pose && has_twist)) {
            loop_rate.sleep();
            continue;
        }

        if (traffic_light_state == "RED") {
            ROS_WARN_THROTTLE(1.0, "Red light detected! Stop!");
            simulation::VehicleControl stop_cmd;
            stop_cmd.Throttle = 0.0;
            stop_cmd.Brake = 1.0;
            stop_cmd.Steering = 0.0;
            cmd_pub.publish(stop_cmd);
            loop_rate.sleep();
            continue;
        }

        // 当前车辆位置和朝向
        float x = current_pose.pose.position.x;
        float y = current_pose.pose.position.y;
        float yaw = getYaw(current_pose.pose.orientation);

        // 目标位置
        float tx = current_target_pose.pose.position.x;
        float ty = current_target_pose.pose.position.y;

        // 计算误差
        float dx = tx - x;
        float dy = ty - y;
        float target_yaw = std::atan2(dy, dx);

        float angle_error = target_yaw - yaw;
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        float distance = std::sqrt(dx * dx + dy * dy);

        // PID 控制速度（纵向）
        float velocity_error = current_velocity.twist.linear.x - current_twist.twist.linear.x;
        integral_lin += velocity_error * loop_interval;
        float derivative_lin = (velocity_error - prev_error_lin) / loop_interval;
        prev_error_lin = velocity_error;
        float throttle_cmd = Kp_lin * velocity_error + Ki_lin * integral_lin + Kd_lin * derivative_lin;

        // PID 控制转向（横向）
        integral_ang += angle_error * loop_interval;
        float derivative_ang = (angle_error - prev_error_ang) / loop_interval;
        prev_error_ang = angle_error;
        float steering_cmd = Kp_ang * angle_error + Ki_ang * integral_ang + Kd_ang * derivative_ang;

        // 控制指令封装
        simulation::VehicleControl cmd;
        cmd.Throttle = std::clamp(throttle_cmd, -1.0f, 1.0f);
        cmd.Steering = std::clamp(steering_cmd, -1.0f, 1.0f);
        cmd.Brake    = (cmd.Throttle < 0) ? -cmd.Throttle : 0.0f;
        cmd.Throttle = std::max(0.0f, cmd.Throttle); // 负数部分由 Brake 承担
        cmd.Reserved = 0.0;

        cmd_pub.publish(cmd);
        loop_rate.sleep();
    }

    return 0;
}
