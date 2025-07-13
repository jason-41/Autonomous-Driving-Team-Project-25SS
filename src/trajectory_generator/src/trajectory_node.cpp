#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

class TrajectoryPlanner {
public:
  TrajectoryPlanner() {
    ros::NodeHandle nh, pnh("~");
    // 车辆参数
    pnh.param("wheel_base", L_, 2.63);
    pnh.param("v_max",     v_max_,    2.0);
    pnh.param("a_max",     a_max_,    1.0);
    pnh.param("a_lat_max", a_lat_max_,0.5);
    pnh.param("time_interval", time_interval_, 0.04);

    path_sub_           = nh.subscribe("planned_path", 1, &TrajectoryPlanner::pathCallback, this);
    current_pose_sub_   = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &TrajectoryPlanner::currentPoseCallback, this);
    vel_pub_            = nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
    pose_pub_           = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
    traj_timer_         = nh.createTimer(ros::Duration(time_interval_), &TrajectoryPlanner::timerCallback, this);

    active_             = false;
    current_pose_received_ = false;
  }

private:
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    const auto &poses = path_msg->poses;
    N_ = poses.size();
    if (N_ < 3) {
      ROS_WARN("需要至少3个路径点来计算曲率，当前：%zu", N_);
      active_ = false;
      return;
    }
    xs_.resize(N_);
    ys_.resize(N_);
    yaws_.resize(N_);
    std::vector<double> ds(N_-1);

    // 提取路径点及航向
    for (int i = 0; i < N_; ++i) {
      xs_[i] = poses[i].pose.position.x;
      ys_[i] = poses[i].pose.position.y;
      if (i > 0) {
        double dx = xs_[i] - xs_[i-1];
        double dy = ys_[i] - ys_[i-1];
        ds[i-1] = std::hypot(dx, dy);
        yaws_[i-1] = std::atan2(dy, dx);
      }
    }
    yaws_[N_-1] = yaws_[N_-2];

    // 计算曲率 k_ 与速度限制 v_limit
    k_.assign(N_, 0.0);
    std::vector<double> v_limit(N_, v_max_);
    for (int i = 1; i < N_-1; ++i) {
      k_[i] = computeCurvature(xs_[i-1], ys_[i-1], xs_[i], ys_[i], xs_[i+1], ys_[i+1]);
      double v_lat = std::sqrt(a_lat_max_ / std::max(std::abs(k_[i]), 1e-6));
      v_limit[i] = std::min(v_max_, v_lat);
    }
    v_limit[0] = v_limit[N_-1] = 0.0;

    // 生成前向/后向加速度限幅的速度剖面 v_
    v_.assign(N_, 0.0);
    for (int i = 1; i < N_; ++i) {
      v_[i] = std::min(v_limit[i], std::sqrt(v_[i-1]*v_[i-1] + 2 * a_max_ * ds[i-1]));
    }
    for (int i = N_-2; i >= 0; --i) {
      v_[i] = std::min(v_[i], std::sqrt(v_[i+1]*v_[i+1] + 2 * a_max_ * ds[i]));
    }

    // 生成时间戳 t_
    t_.assign(N_, 0.0);
    for (int i = 1; i < N_; ++i) {
      double dt = (v_[i] > 1e-3) ? (ds[i-1] / v_[i]) : time_interval_;
      t_[i] = t_[i-1] + dt;
    }

    frame_id_ = path_msg->header.frame_id;
    cur_idx_  = 0;
    start_    = ros::Time::now();
    active_   = true;
  }

  void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    current_pose_received_ = true;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!active_ || !current_pose_received_) return;

    // 1) 时间计算
    ros::Time now = ros::Time::now();
    double elapsed = (now - start_).toSec();

    // 2) 最近点匹配，修正 cur_idx_
    double cx = current_pose_.pose.position.x;
    double cy = current_pose_.pose.position.y;
    double min_d2 = std::numeric_limits<double>::infinity();
    size_t best_i = cur_idx_;
    for (size_t j = cur_idx_; j < N_; ++j) {
      double dx = xs_[j] - cx;
      double dy = ys_[j] - cy;
      double d2 = dx*dx + dy*dy;
      if (d2 < min_d2) {
        min_d2 = d2;
        best_i = j;
      } else {
        break;
      }
    }
    cur_idx_ = best_i;

    // 3) 时间推进
    while (cur_idx_ + 1 < N_ && elapsed > t_[cur_idx_+1]) {
      ++cur_idx_;
    }
    if (cur_idx_ + 1 >= N_) {
      publishStop();
      active_ = false;
      return;
    }

    // 4) 重新对齐 start_
    start_ = now - ros::Duration(t_[cur_idx_]);

    // 5) 插值计算
    double dt_seg = t_[cur_idx_+1] - t_[cur_idx_];
    double alpha = (dt_seg > 1e-6) ? (elapsed - t_[cur_idx_]) / dt_seg : 0.0;

    double x       = xs_[cur_idx_] + alpha * (xs_[cur_idx_+1] - xs_[cur_idx_]);
    double y       = ys_[cur_idx_] + alpha * (ys_[cur_idx_+1] - ys_[cur_idx_]);
    double yaw     = yaws_[cur_idx_] + alpha * (yaws_[cur_idx_+1] - yaws_[cur_idx_]);
    double vel_lin = v_[cur_idx_]   + alpha * (v_[cur_idx_+1]   - v_[cur_idx_]);

    // 发布 target_pose
    geometry_msgs::PoseStamped ps;
    ps.header.stamp    = now;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_pub_.publish(ps);

    // 发布速度指令
    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp    = now;
    cmd.header.frame_id = frame_id_;
    cmd.twist.linear.x  = vel_lin;
    cmd.twist.angular.z = vel_lin * k_[cur_idx_];
    vel_pub_.publish(cmd);
  }

  void publishStop() {
    geometry_msgs::TwistStamped stop;
    stop.header.stamp    = ros::Time::now();
    stop.twist.linear.x  = 0.0;
    stop.twist.angular.z = 0.0;
    vel_pub_.publish(stop);

    geometry_msgs::PoseStamped last;
    last.header.stamp    = ros::Time::now();
    last.header.frame_id = frame_id_;
    last.pose.position.x = xs_.back();
    last.pose.position.y = ys_.back();
    last.pose.orientation = tf::createQuaternionMsgFromYaw(yaws_.back());
    pose_pub_.publish(last);
  }

  double computeCurvature(double x0, double y0,
                          double x1, double y1,
                          double x2, double y2) {
    double a = std::hypot(x1-x0, y1-y0);
    double b = std::hypot(x2-x1, y2-y1);
    double c = std::hypot(x2-x0, y2-y0);
    double area = std::abs((x1-x0)*(y2-y0) - (x2-x0)*(y1-y0)) * 0.5;
    if (a * b * c < 1e-6) return 0.0;
    return 4.0 * area / (a * b * c);
  }

private:
  ros::Subscriber            path_sub_, current_pose_sub_;
  ros::Publisher             vel_pub_, pose_pub_;
  ros::Timer                 traj_timer_;

  std::vector<double>        xs_, ys_, yaws_, v_, k_, t_;
  size_t                     cur_idx_{0}, N_{0};
  bool                       active_{false};
  ros::Time                  start_;
  std::string                frame_id_;

  // 实时反馈
  geometry_msgs::PoseStamped current_pose_;
  bool                       current_pose_received_{false};

  // 参数
  double L_, v_max_, a_max_, a_lat_max_, time_interval_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generator_node_v2");
  TrajectoryPlanner planner;
  ros::spin();
  return 0;
}
