#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

class TrajectoryGenerator {
public:
    TrajectoryGenerator() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 1, true);
        path_sub_ = nh.subscribe("planned_path", 1, &TrajectoryGenerator::pathCallback, this);
        pnh.param("time_interval", time_interval_, 0.1);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
        if (path_msg->poses.empty()) return;
        trajectory_msgs::MultiDOFJointTrajectory traj;
        traj.header.frame_id = path_msg->header.frame_id;
        traj.header.stamp = ros::Time::now();
        traj.joint_names.push_back("base_link");
        geometry_msgs::Transform prev_tf;
        geometry_msgs::Twist prev_vel;
        bool first = true;
        ros::Duration t(0.0);
        for (size_t i = 0; i < path_msg->poses.size(); ++i) {
            const auto& pose = path_msg->poses[i];
            geometry_msgs::Transform tf;
            tf.translation.x = pose.pose.position.x;
            tf.translation.y = pose.pose.position.y;
            tf.translation.z = pose.pose.position.z;
            tf.rotation = pose.pose.orientation;
            geometry_msgs::Twist vel;
            geometry_msgs::Twist acc;
            if (first) {
                vel.linear.x = vel.linear.y = vel.linear.z = 0;
                vel.angular.x = vel.angular.y = vel.angular.z = 0;
                acc.linear.x = acc.linear.y = acc.linear.z = 0;
                acc.angular.x = acc.angular.y = acc.angular.z = 0;
                first = false;
            } else {
                double dt = time_interval_;
                vel.linear.x = (tf.translation.x - prev_tf.translation.x) / dt;
                vel.linear.y = (tf.translation.y - prev_tf.translation.y) / dt;
                vel.linear.z = (tf.translation.z - prev_tf.translation.z) / dt;
                vel.angular.x = vel.angular.y = vel.angular.z = 0;
                acc.linear.x = (vel.linear.x - prev_vel.linear.x) / dt;
                acc.linear.y = (vel.linear.y - prev_vel.linear.y) / dt;
                acc.linear.z = (vel.linear.z - prev_vel.linear.z) / dt;
                acc.angular.x = acc.angular.y = acc.angular.z = 0; // Assuming zero angular acceleration for simplicity 孔说先不用加速度信息
            }
            trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
            pt.transforms.push_back(tf);
            pt.velocities.push_back(vel);
            pt.accelerations.push_back(acc);
            pt.time_from_start = t;
            traj.points.push_back(pt);
            prev_tf = tf;
            prev_vel = vel;
            t += ros::Duration(time_interval_);
        }
        traj_pub_.publish(traj);
    }

private:
    ros::Subscriber path_sub_;
    ros::Publisher traj_pub_;
    double time_interval_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_node");
    TrajectoryGenerator tg;
    ros::spin();
    return 0;
}
