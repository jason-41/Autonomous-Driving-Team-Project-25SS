#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <limits>

class TrajectoryPlanner {
public:
  TrajectoryPlanner() {
    ros::NodeHandle nh, pnh("~");
    // 车辆参数
    pnh.param("wheel_base",     L_,             2.63);
    pnh.param("v_max",          v_max_,         8.0);
    pnh.param("a_max",          a_max_,         500.0);
    pnh.param("a_lat_max",      a_lat_max_,     500.0);
    pnh.param("time_interval",  time_interval_, 0.1);
    pnh.param("lookahead_dist", lookahead_dist_, 1.0);
    // 保持速度和停止参数
    pnh.param("hold_dist",      hold_dist_,      1.0);
    pnh.param("stop_after_dist", stop_after_dist_, 3.0);

    path_sub_         = nh.subscribe("planned_path", 1, &TrajectoryPlanner::pathCallback, this);
    current_pose_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1,
                                     &TrajectoryPlanner::currentPoseCallback, this);
    vel_pub_          = nh.advertise<geometry_msgs::TwistStamped>("velocity", 1);
    pose_pub_         = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    traj_timer_       = nh.createTimer(ros::Duration(time_interval_),
                                       &TrajectoryPlanner::timerCallback, this);

    active_ = false;
    current_pose_received_ = false;
  }

private:
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    const auto& poses = path_msg->poses;
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

    for (size_t i = 0; i < N_; ++i) {
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

    // 曲率与速度限幅
    k_.assign(N_, 0.0);
    std::vector<double> v_limit(N_, v_max_);
    for (size_t i = 1; i < N_-1; ++i) {
      k_[i] = computeCurvature(
        xs_[i-1], ys_[i-1], xs_[i], ys_[i], xs_[i+1], ys_[i+1]);
      double v_lat = std::sqrt(a_lat_max_ / std::max(std::abs(k_[i]), 1e-6));
      v_limit[i] = std::min(v_max_, v_lat);
    }
    v_limit[0] = v_limit[N_-1] = 0.0;

    // 加速度限幅速度
    v_.assign(N_, 0.0);
    for (size_t i = 1; i < N_; ++i) {
      v_[i] = std::min(v_limit[i],
        std::sqrt(v_[i-1]*v_[i-1] + 2 * a_max_ * ds[i-1]));
    }
    for (int i = static_cast<int>(N_)-2; i >= 0; --i) {
      v_[i] = std::min(v_[i],
        std::sqrt(v_[i+1]*v_[i+1] + 2 * a_max_ * ds[i]));
    }

    // 时间戳
    t_.assign(N_, 0.0);
    for (size_t i = 1; i < N_; ++i) {
      double dt = (v_[i] > 1e-3) ? ds[i-1]/v_[i] : time_interval_;
      t_[i] = t_[i-1] + dt;
    }

    frame_id_ = path_msg->header.frame_id;
    cur_idx_  = 0;
    start_    = ros::Time::now();
    active_   = true;

    // 终点与保持逻辑
    end_x_ = xs_.back(); end_y_ = ys_.back();
    hold_flag_ = false; hold_vel_ = 0.0;
    reached_flag_ = false;

    ROS_INFO_STREAM("Received path with " << N_ << " points. "
      << "Start: (" << xs_.front() << ", " << ys_.front() << ")"
      << ", End: (" << xs_.back() << ", " << ys_.back() << ")");
  }

  void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    current_pose_received_ = true;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!active_ || !current_pose_received_) return;

    ros::Time now = ros::Time::now();
    double elapsed = (now - start_).toSec();

    // 最近点匹配与推进索引
    double cx = current_pose_.pose.position.x;
    double cy = current_pose_.pose.position.y;
    double min_d2 = std::numeric_limits<double>::infinity();
    size_t best_i = cur_idx_;
    
    // 限定搜索窗口，防止回跳过多
    size_t search_start = (cur_idx_ > 5) ? cur_idx_ - 5 : 0;
    size_t search_end = std::min(N_, cur_idx_ + 20);

    for (size_t j = search_start; j < search_end; ++j) {
      double dx = xs_[j] - cx;
      double dy = ys_[j] - cy;
      double d2 = dx * dx + dy * dy;
    
      // 可加入朝向判断：当前朝向与路径切线夹角过大不接受
      if (d2 < min_d2) {
        min_d2 = d2;
        best_i = j;
      }
    }
    cur_idx_ = std::max(best_i, cur_idx_); // 只允许前进
    
    // 推进 index，确保时间推进或路径推进
    while (cur_idx_ + 1 < N_ && elapsed > t_[cur_idx_ + 1]) ++cur_idx_;

    double vel_lin;
    if (cur_idx_+1 < N_) {
      // 插值计算
      double dt_seg = t_[cur_idx_+1] - t_[cur_idx_];
      double alpha = (dt_seg>1e-6) ? (elapsed - t_[cur_idx_]) / dt_seg : 0.0;
      vel_lin = v_[cur_idx_] + alpha * (v_[cur_idx_+1] - v_[cur_idx_]);

      double dist_end = std::hypot(cx - end_x_, cy - end_y_);
      if (dist_end <= hold_dist_) {
        if (!hold_flag_) { hold_flag_ = true; hold_vel_ = vel_lin; }
        vel_lin = hold_vel_;
      }
      else {
        hold_flag_ = false;
      }
    } else {
      // 到达终点
      if (!reached_flag_) {
        reached_flag_ = true;
        reach_x_ = cx; reach_y_ = cy;
      }
      double travelled = std::hypot(cx - reach_x_, cy - reach_y_);
      if (travelled >= stop_after_dist_) {
        publishStop(); active_ = false; return;
      }
      vel_lin = hold_vel_;
    }

    // 发布 target_pose
    size_t look_i = cur_idx_;
    for (size_t i = cur_idx_; i < N_; ++i) {
      double dx = xs_[i] - cx, dy = ys_[i] - cy;
      if (std::hypot(dx, dy) >= lookahead_dist_) { look_i = i; break; }
    }
    if (look_i >= N_) look_i = N_-1;
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = now; ps.header.frame_id = frame_id_;
    ps.pose.position.x = xs_[look_i]; ps.pose.position.y = ys_[look_i];
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaws_[look_i]);
    pose_pub_.publish(ps);

    // 发布速度
    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp = now; cmd.header.frame_id = frame_id_;
    cmd.twist.linear.x = vel_lin;
    cmd.twist.angular.z = vel_lin * k_[std::min(cur_idx_, N_-1)];
    vel_pub_.publish(cmd);

    double yaw = tf::getYaw(current_pose_.pose.orientation);
    ROS_INFO_STREAM("Current position: x = " << cx 
                                        << ", y = " << cy
                                        << ", yaw = " << yaw    
                                        << ", velocity = " << vel_lin);

    ROS_INFO_STREAM("Target pose: x = " << ps.pose.position.x 
                                        << ", y = " << ps.pose.position.y 
                                        << ", orientation = " << tf::getYaw(ps.pose.orientation));
    ROS_INFO_STREAM("Target velocity: linear = " << cmd.twist.linear.x 
                                                  << ", angular = " << cmd.twist.angular.z);
    ROS_INFO_STREAM("Current curvature: " << k_[cur_idx_]
                                          << ", Velocity limit: " << v_[cur_idx_]);
  }

  void publishStop() {
    geometry_msgs::TwistStamped stop;
    stop.header.stamp = ros::Time::now();
    stop.twist.linear.x = 0.0; stop.twist.angular.z = 0.0;
    vel_pub_.publish(stop);

    geometry_msgs::PoseStamped last;
    last.header.stamp = ros::Time::now(); last.header.frame_id = frame_id_;
    last.pose.position.x = end_x_; last.pose.position.y = end_y_;
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
    if (a*b*c < 1e-6) return 0.0;
    return 4.0*area/(a*b*c);
  }

private:
  ros::Subscriber            path_sub_, current_pose_sub_;
  ros::Publisher             vel_pub_, pose_pub_;
  ros::Timer                 traj_timer_;

  std::vector<double>        xs_, ys_, yaws_, v_, k_, t_;
  size_t                     cur_idx_{0}, N_{0};
  bool                       active_{false}, current_pose_received_{false};
  ros::Time                  start_;
  std::string                frame_id_;

  double                     lookahead_dist_, hold_dist_, stop_after_dist_;
  double                     end_x_, end_y_, reach_x_, reach_y_, hold_vel_;
  bool                       hold_flag_, reached_flag_;
  double L_;
  double v_max_;
  double a_max_;
  double a_lat_max_;
  double time_interval_;

  geometry_msgs::PoseStamped current_pose_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generator_node");
  TrajectoryPlanner planner;
  ros::spin();
  return 0;
}
