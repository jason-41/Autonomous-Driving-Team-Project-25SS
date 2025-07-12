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
    // 车辆几何与动力学参数（wheel_base 未在此使用，可按需扩展）
    pnh.param("wheel_base", L_, 2.63);
    pnh.param("v_max",     v_max_,    2.0);
    pnh.param("a_max",     a_max_,    1.0);
    pnh.param("a_lat_max", a_lat_max_,0.5);
    pnh.param("time_interval", time_interval_, 0.04);

    path_sub_    = nh.subscribe("planned_path", 1, &TrajectoryPlanner::pathCallback, this);
    vel_pub_     = nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
    pose_pub_    = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
    traj_timer_  = nh.createTimer(
      ros::Duration(time_interval_),
      &TrajectoryPlanner::timerCallback,
      this);

    active_ = false;
  }

private:
  // 回调：接收一次完整路径并预计算所有插值数据
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    const auto &poses = path_msg->poses;
    N_ = poses.size();
    if (N_ < 3) {
      ROS_WARN("需要至少3个路径点来计算曲率，当前：%d", N_);
      active_ = false;
      return;
    }
    xs_.resize(N_);
    ys_.resize(N_);
    yaws_.resize(N_);
    std::vector<double> ds(N_-1);

    // 提取点坐标及航向
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

    // 计算曲率 k_ 和速度上限 v_limit
    k_.assign(N_, 0.0);
    std::vector<double> v_limit(N_, v_max_);
    for (int i = 1; i < N_-1; ++i) {
      k_[i] = computeCurvature(xs_[i-1], ys_[i-1], xs_[i], ys_[i], xs_[i+1], ys_[i+1]);
      double v_lat = std::sqrt(a_lat_max_ / std::max(std::abs(k_[i]), 1e-6));
      v_limit[i] = std::min(v_max_, v_lat);
    }
    v_limit[0] = v_limit[N_-1] = 0.0;

    // 前后向加速度限幅，生成速度剖面 v_
    v_.assign(N_, 0.0);
    for (int i = 1; i < N_; ++i) {
      v_[i] = std::min(v_limit[i], std::sqrt(v_[i-1]*v_[i-1] + 2 * a_max_ * ds[i-1]));
    }
    for (int i = N_-2; i >= 0; --i) {
      v_[i] = std::min(v_[i], std::sqrt(v_[i+1]*v_[i+1] + 2 * a_max_ * ds[i]));
    }

    // 时间戳分配 t_
    t_.assign(N_, 0.0);
    for (int i = 1; i < N_; ++i) {
      double dt = (v_[i] > 1e-3) ? (ds[i-1] / v_[i]) : time_interval_;
      t_[i] = t_[i-1] + dt;
    }

    frame_id_   = path_msg->header.frame_id;
    cur_idx_    = 0;
    start_      = ros::Time::now();
    active_     = true;
  }

  // 定时器回调：按 time_interval_ 发布插值速度与位姿
  void timerCallback(const ros::TimerEvent&) {
    if (!active_) return;
    double elapsed = (ros::Time::now() - start_).toSec();

    // 定位当前段索引 cur_idx_
    while (cur_idx_ + 1 < N_ && elapsed > t_[cur_idx_+1]) {
      ++cur_idx_;
    }
    if (cur_idx_ + 1 >= N_) {
      publishStop();
      active_ = false;
      return;
    }

    double dt_seg = t_[cur_idx_+1] - t_[cur_idx_];
    double alpha  = (dt_seg > 1e-6)
                    ? (elapsed - t_[cur_idx_]) / dt_seg
                    : 0.0;

    // 插值计算
    double x       = xs_[cur_idx_] + alpha * (xs_[cur_idx_+1] - xs_[cur_idx_]);
    double y       = ys_[cur_idx_] + alpha * (ys_[cur_idx_+1] - ys_[cur_idx_]);
    double yaw     = yaws_[cur_idx_] + alpha * (yaws_[cur_idx_+1] - yaws_[cur_idx_]);
    double vel_lin = v_[cur_idx_]   + alpha * (v_[cur_idx_+1]   - v_[cur_idx_]);

    // 发布目标位姿
    geometry_msgs::PoseStamped ps;
    ps.header.stamp    = ros::Time::now();
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_pub_.publish(ps);

    // 发布速度指令
    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp    = ps.header.stamp;
    cmd.header.frame_id = frame_id_;
    cmd.twist.linear.x  = vel_lin;
    cmd.twist.angular.z = vel_lin * k_[cur_idx_];
    vel_pub_.publish(cmd);
  }

  // 发布停止指令和最后位置
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

  // 三点法计算曲率
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

  // 成员变量
  ros::Subscriber            path_sub_;
  ros::Publisher             vel_pub_, pose_pub_;
  ros::Timer                 traj_timer_;

  std::vector<double>        xs_, ys_, yaws_, v_, k_, t_;
  size_t                     cur_idx_{0}, N_{0};
  bool                       active_{false};
  ros::Time                  start_;
  std::string                frame_id_;

  double                     L_, v_max_, a_max_, a_lat_max_, time_interval_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generator_node_v2");
  TrajectoryPlanner planner;
  ros::spin();
  return 0;
}
