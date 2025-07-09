#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <algorithm>

class TrajectoryPlanner {
public:
  TrajectoryPlanner() {
    ros::NodeHandle nh, pnh("~");
    // 车辆几何与动力学参数
    pnh.param("wheel_base", L_, 2.63);
    pnh.param("v_max",     v_max_,    2.0);
    pnh.param("a_max",     a_max_,    1.0);
    pnh.param("a_lat_max", a_lat_max_,0.5);
    time_interval_ = pnh.param("time_interval", 0.1); // 默认 0.1 秒

    path_sub_  = nh.subscribe("planned_path", 1, &TrajectoryPlanner::pathCallback, this);
    vel_pub_   = nh.advertise<geometry_msgs::TwistStamped>("velocity", 1);
    pose_pub_  = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
  }

private:
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    const auto &poses = path_msg->poses;
    int N = poses.size();
    if (N < 3) {
      ROS_WARN("需要至少3个路径点来计算曲率，当前：%d", N);
      return;
    }

    // 1. 提取离散点及航向
    std::vector<double> xs(N), ys(N), yaws(N), ds(N-1);
    for (int i = 0; i < N; ++i) {
      xs[i]   = poses[i].pose.position.x;
      ys[i]   = poses[i].pose.position.y;
      // yaws[i] = tf::getYaw(poses[i].pose.orientation); A* 规划器来的路径点并没有设置真实 orientation，所以全部是默认的 yaw = 0，也就是四元数 (0,0,0,1)

      if (i > 0) {
        double dx = xs[i] - xs[i-1];
        double dy = ys[i] - ys[i-1];
        ds[i-1]   = std::hypot(dx, dy);

        // 用当前位置和前一个位置计算方向
        yaws[i-1] = std::atan2(dy, dx);
      }
    }
    yaws[N-1] = yaws[N-2]; // 最后一个点无法计算方向，设定最后一个点的方向与倒数第二个点相同

    // 2. 计算曲率 k 和速度上限 v_limit
    std::vector<double> k(N, 0.0), v_limit(N, v_max_);
    for (int i = 1; i < N-1; ++i) {
      k[i] = computeCurvature(xs[i-1], ys[i-1], xs[i], ys[i], xs[i+1], ys[i+1]);
      double v_lat = std::sqrt(a_lat_max_ / std::max(std::abs(k[i]), 1e-6));
      v_limit[i]   = std::min(v_max_, v_lat);
    }
    v_limit[0] = v_limit[N-1] = 0.0;

    // 3. 前后向加速度限幅，生成速度剖面 v[i]
    std::vector<double> v(N, 0.0);
    for (int i = 1; i < N; ++i) {
      v[i] = std::min(v_limit[i],
                      std::sqrt(v[i-1]*v[i-1] + 2 * a_max_ * ds[i-1]));
    }
    for (int i = N-2; i >= 0; --i) {
      v[i] = std::min(v[i],
                      std::sqrt(v[i+1]*v[i+1] + 2 * a_max_ * ds[i]));
    }

    // 4. 时间戳分配 t[i]
    std::vector<double> t(N, 0.0);
    for (int i = 1; i < N; ++i) {
      double dt = (v[i] > 1e-3) ? (ds[i-1] / v[i]) : time_interval_;
      t[i] = t[i-1] + dt;
    }

    // 5. 按时间间隔发布速度与位置
    ros::Time start = ros::Time::now();
    size_t idx = 0;
    ros::Rate rate(1.0 / time_interval_);
    while (ros::ok() && idx < poses.size()) {
      ros::Duration elapsed = ros::Time::now() - start;
      if (elapsed.toSec() >= t[idx]) {
        // 发布速度
        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp    = ros::Time::now();
        cmd.header.frame_id = path_msg->header.frame_id;
        cmd.twist.linear.x  = v[idx];
        cmd.twist.angular.z = v[idx] * k[idx];
        vel_pub_.publish(cmd);

        // 发布位置
        geometry_msgs::PoseStamped target;
        target.header.stamp    = ros::Time::now();
        target.header.frame_id = path_msg->header.frame_id;
        target.pose.position.x = xs[idx];
        target.pose.position.y = ys[idx];
        target.pose.orientation = tf::createQuaternionMsgFromYaw(yaws[idx]);
        pose_pub_.publish(target);

        idx++;
      }
      rate.sleep();
    }

    // 路径结束，发送停止指令和最后位置
    geometry_msgs::TwistStamped stop;
    stop.header.stamp    = ros::Time::now();
    stop.twist.linear.x  = 0.0;
    stop.twist.angular.z = 0.0;
    vel_pub_.publish(stop);

    geometry_msgs::PoseStamped last;
    last.header.stamp    = ros::Time::now();
    last.header.frame_id = path_msg->header.frame_id;
    last.pose.position.x = xs.back();
    last.pose.position.y = ys.back();
    last.pose.orientation = tf::createQuaternionMsgFromYaw(yaws.back());
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

  ros::Subscriber              path_sub_;
  ros::Publisher               vel_pub_;
  ros::Publisher               pose_pub_;
  double                       L_, v_max_, a_max_, a_lat_max_, time_interval_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generator_node_v2");
  TrajectoryPlanner planner;
  ros::spin();
  return 0;
}
