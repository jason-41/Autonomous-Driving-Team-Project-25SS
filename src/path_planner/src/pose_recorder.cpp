#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <cmath>

class PoseRecorder {
public:
    PoseRecorder() : first_msg_received_(false), last_x_(0.0), last_y_(0.0) {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &PoseRecorder::poseCallback, this);

        // 打开文件，准备写入（追加模式）
        file_.open("/tmp/car_positions.txt", std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            ROS_ERROR("Failed to open file for writing.");
            ros::shutdown();
        } else {
            // 写入表头（如果文件为空）
            file_ << "x,y\n";
            file_.flush();
        }
    }

    ~PoseRecorder() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    ros::Subscriber sub_;
    std::ofstream file_;
    bool first_msg_received_;
    double last_x_, last_y_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        if (!first_msg_received_) {
            // 第一次收到消息，直接记录
            last_x_ = x;
            last_y_ = y;
            first_msg_received_ = true;
            recordPosition(x, y);
            ROS_INFO("Initial position recorded: (%f, %f)", x, y);
            return;
        }

        double dist = std::hypot(x - last_x_, y - last_y_);
        if (dist >= 10.0) {  // 移动超过10米才记录
            last_x_ = x;
            last_y_ = y;
            recordPosition(x, y);
            ROS_INFO("Moved 10m+, position recorded: (%f, %f)", x, y);
        }
    }

    void recordPosition(double x, double y) {
        if (file_.is_open()) {
            file_ << x << "," << y << "\n";
            file_.flush();  // 立即写入文件，防止丢失
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_recorder_node");
    PoseRecorder recorder;
    ros::spin();
    return 0;
}
