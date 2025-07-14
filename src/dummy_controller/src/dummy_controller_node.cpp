#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <simulation/VehicleControl.h>
#include <tf/tf.h>
#include <cmath>
#include <algorithm>  // for std::clamp
#include <iomanip>

class DummyPIDController
{
public:
    DummyPIDController()
        : speed_integral_(0.0), steer_integral_(0.0),
          prev_speed_error_(0.0), prev_steer_error_(0.0),
          current_pose_received_(false), current_twist_received_(false),
          target_pose_received_(false), target_velocity_received_(false),
          traffic_state_("GREEN"), prev_throttle_(0.0), prev_steering_(0.0), alpha_filter_(0.2)
    {
        ros::NodeHandle nh; ros::NodeHandle pnh("~");
        // 参数: 滤波系数
        pnh.param("alpha_filter", alpha_filter_, alpha_filter_);

        // === 订阅器 ===
        pose_sub_     = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &DummyPIDController::poseCallback, this);
        twist_sub_    = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 10, &DummyPIDController::twistCallback, this);
        target_sub_   = nh.subscribe("/target_pose", 10, &DummyPIDController::targetCallback, this);
        velocity_sub_ = nh.subscribe("/velocity", 10, &DummyPIDController::velocityCallback, this);
        traffic_sub_  = nh.subscribe("/traffic_state", 1, &DummyPIDController::trafficCallback, this);

        cmd_pub_      = nh.advertise<simulation::VehicleControl>("car_command", 1);
    }

    void run()
    {
        ros::Rate loop_rate(10);
        float dt = 0.1;

        while (ros::ok())
        {
            simulation::VehicleControl cmd;
            cmd.Reserved = 0.0;
            ros::spinOnce();

            if (!current_pose_received_ || !current_twist_received_ || !target_pose_received_ || !target_velocity_received_) {
                ROS_WARN_THROTTLE(1.0, "Waiting for pose/twist/target_pose/velocity");
                cmd_pub_.publish(cmd);
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            // === PID 控制速度 ===
            double speed_error = target_velocity_.twist.linear.x - current_twist_.twist.linear.x;
            speed_integral_ += speed_error * dt;
            double speed_derivative = (speed_error - prev_speed_error_) / dt;
            prev_speed_error_ = speed_error;
            double Kp_v = 40, Ki_v = 0.0, Kd_v = 0.0;
            if (speed_error >= 0) {
                cmd.Throttle = Kp_v * speed_error + Ki_v * speed_integral_ + Kd_v * speed_derivative;
                cmd.Throttle = std::clamp(cmd.Throttle, 0.0f, 0.5f);
                cmd.Brake = 0.0;
            } else {
                cmd.Throttle = 0.0;
                cmd.Brake = std::clamp(0.8 * (-speed_error), 0.0, 1.0);
            }

            // === PID 控制方向 ===
            double dx = target_pose_.pose.position.x - current_pose_.pose.position.x;
            double dy = target_pose_.pose.position.y - current_pose_.pose.position.y;
            double current_yaw = tf::getYaw(current_pose_.pose.orientation);
            double desired_yaw = std::atan2(dy, dx);
            double steer_error = current_yaw - desired_yaw;
            while (steer_error > M_PI)  steer_error -= 2 * M_PI;
            while (steer_error < -M_PI) steer_error += 2 * M_PI;
            steer_integral_ += steer_error * dt;
            double steer_derivative = (steer_error - prev_steer_error_) / dt;
            prev_steer_error_ = steer_error;
            double Kp_s = 20, Ki_s = 0.1, Kd_s = 0.05;
            cmd.Steering = Kp_s * steer_error + Ki_s * steer_integral_ + Kd_s * steer_derivative;
            cmd.Steering = std::clamp(cmd.Steering, -1.0f, 1.0f);

            // === 一阶滤波平滑突变 ===
            cmd.Throttle  = alpha_filter_ * cmd.Throttle  + (1.0 - alpha_filter_) * prev_throttle_;
            cmd.Steering  = alpha_filter_ * cmd.Steering  + (1.0 - alpha_filter_) * prev_steering_;
            prev_throttle_ = cmd.Throttle;
            prev_steering_ = cmd.Steering;

            // 插入调试输出
            ROS_INFO_STREAM("Publishing cmd => Throttle: " << cmd.Throttle
                            << ", Brake: " << cmd.Brake
                            << ", Steering: " << cmd.Steering);
            cmd_pub_.publish(cmd);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::Subscriber           pose_sub_;
    ros::Subscriber           twist_sub_;
    ros::Subscriber           target_sub_;
    ros::Subscriber           velocity_sub_;
    ros::Subscriber           traffic_sub_;

    ros::Publisher            cmd_pub_;

    geometry_msgs::PoseStamped current_pose_, target_pose_;
    geometry_msgs::TwistStamped current_twist_, target_velocity_;

    bool                      current_pose_received_;
    bool                      current_twist_received_;
    bool                      target_pose_received_;
    bool                      target_velocity_received_;

    std::string               traffic_state_;

    // PID 状态
    double                    speed_integral_, steer_integral_;
    double                    prev_speed_error_, prev_steer_error_;

    // 滤波状态
    double                    prev_throttle_, prev_steering_, alpha_filter_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        current_pose_received_ = true;
    }
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_twist_ = *msg;
        current_twist_received_ = true;
    }
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        target_pose_ = *msg;
        target_pose_received_ = true;
    }
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        target_velocity_ = *msg;
        target_velocity_received_ = true;
    }
    void trafficCallback(const std_msgs::String::ConstPtr& msg) {
        traffic_state_ = msg->data;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_controller_node");
    DummyPIDController controller;
    controller.run();
    return 0;
}
