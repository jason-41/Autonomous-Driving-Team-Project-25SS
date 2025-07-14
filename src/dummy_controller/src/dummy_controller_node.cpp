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
          target_pose_received_(false), target_velocity_received_(false), traffic_state_("GREEN")
    {
        ros::NodeHandle nh;

        // === 订阅器 ===
        pose_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &DummyPIDController::poseCallback, this);
        twist_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 1, &DummyPIDController::twistCallback, this);
        target_sub_ = nh.subscribe("/target_pose", 1, &DummyPIDController::targetCallback, this);
        velocity_sub_ = nh.subscribe("/velocity", 1, &DummyPIDController::velocityCallback, this);
        traffic_sub_ = nh.subscribe("/traffic_state", 1, &DummyPIDController::trafficCallback, this);

        cmd_pub_ = nh.advertise<simulation::VehicleControl>("car_command", 1);
    }

    void run()
    {
        ros::Rate loop_rate(10);
        float dt = 0.1;

        while (ros::ok())
        {
            simulation::VehicleControl cmd;
            cmd.Reserved = 0.0;

            if (!current_pose_received_ || !current_twist_received_ || !target_pose_received_) {
                ROS_WARN_THROTTLE(1.0, "Waiting for pose/twist/target_pose/velocity");
                cmd_pub_.publish(cmd);
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            if (traffic_state_ == "RED") {
                cmd.Throttle = 0.0;
                cmd.Brake = 1.0;
                cmd.Steering = 0.0;
            } else {
              // === 判断目标是否在车后方 ===
                double dx = target_pose_.pose.position.x - current_pose_.pose.position.x;
                double dy = target_pose_.pose.position.y - current_pose_.pose.position.y;
                double current_yaw = tf::getYaw(current_pose_.pose.orientation);
                double heading_x = std::cos(current_yaw);
                double heading_y = std::sin(current_yaw);
                double dot = dx * heading_x + dy * heading_y;

                if (dot < 0) {
                    ROS_WARN_STREAM("The target point is behind the car, ignore it");
                    cmd.Throttle = 0.0;
                    cmd.Brake = 1.0;
                    cmd.Steering = 0.0;
                    cmd_pub_.publish(cmd);
                    ros::spinOnce();
                    loop_rate.sleep();
                    continue;
                }
                // === PID 控制速度 ===
                double target_speed = target_velocity_.twist.linear.x;
                double current_speed = current_twist_.twist.linear.x;
                double speed_error = target_speed - current_speed;

                speed_integral_ += speed_error * dt;
                double speed_derivative = (speed_error - prev_speed_error_) / dt;
                prev_speed_error_ = speed_error;

                double Kp_v = 40.0, Ki_v = 0.1, Kd_v = 0.01;

               if (speed_error >= 0) {
                  // 当前车速低于目标速度：给油门
                  cmd.Throttle = Kp_v * speed_error + Ki_v * speed_integral_ + Kd_v * speed_derivative;
                  cmd.Throttle = std::clamp(cmd.Throttle, 0.0f, 0.6f);
                  cmd.Brake = 0.0;
                } else {
                  // 当前车速高于目标速度：刹车
                  cmd.Throttle = 0.0;
                  cmd.Brake = 0.0;

                }


                // === PID 控制方向 ===
                double desired_yaw = std::atan2(dy, dx);
                double steer_error = current_yaw - desired_yaw;//negative number turns left, positive number turns right

                while (steer_error > M_PI) steer_error -= 2 * M_PI;
                while (steer_error < -M_PI) steer_error += 2 * M_PI;
                // 插入调试输出
                ROS_INFO_STREAM("current_yaw: " << current_yaw 
                             << ", desired_yaw: " << desired_yaw 
                             << ", steer_error: " << steer_error);
                steer_integral_ += steer_error * dt;
                double steer_derivative = (steer_error - prev_steer_error_) / dt;
                prev_steer_error_ = steer_error;

                double Kp_s = 1.0, Ki_s = 0.1, Kd_s = 0.05;
                cmd.Steering = Kp_s * steer_error + Ki_s * steer_integral_ + Kd_s * steer_derivative;
                cmd.Steering = std::clamp(cmd.Steering, -1.0f, 1.0f);
                 // 插入调试输出
                 ROS_INFO_STREAM("Steer error: " << steer_error 
                            << ", P: " << Kp_s * steer_error 
                            << ", D: " << Kd_s * steer_derivative 
                            << ", Steering(before clamp): " 
                            << (Kp_s * steer_error + Kd_s * steer_derivative));
            }
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
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber traffic_sub_;

    ros::Publisher cmd_pub_;

    geometry_msgs::PoseStamped current_pose_, target_pose_;
    geometry_msgs::TwistStamped current_twist_, target_velocity_;

    bool current_pose_received_;
    bool current_twist_received_;
    bool target_pose_received_;
    bool target_velocity_received_;

    std::string traffic_state_;


    double speed_integral_, steer_integral_;
    double prev_speed_error_, prev_steer_error_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        current_pose_received_ = true;
        ROS_INFO_STREAM("Pose received: x = " << msg->pose.position.x << ", y = " << msg->pose.position.y);
    }

    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_twist_ = *msg;
        current_twist_received_ = true;
        ROS_INFO_STREAM("twist received! speed x: " << msg->twist.linear.x);
    }

    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        target_pose_ = *msg;
        target_pose_received_ = true;
        ROS_INFO_STREAM(std::fixed << std::setprecision(4) << "Target pose received: x = " << msg->pose.position.x << ", y = " << msg->pose.position.y);
    }
    
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        target_velocity_ = *msg;
        target_velocity_received_ = true;
        ROS_INFO_STREAM("Target velocity received (after clamp): x = " << target_velocity_.twist.linear.x);
    }
   
    void trafficCallback(const std_msgs::String::ConstPtr& msg) {
        traffic_state_ = msg->data;
        ROS_INFO_STREAM("Traffic light state received: " << traffic_state_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_controller_node");
    DummyPIDController controller;
    controller.run();
    return 0;
}
