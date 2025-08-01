#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <cmath>

class PoseRecorder {
public:
    PoseRecorder() : first_msg_received_(false), last_x_(0.0), last_y_(0.0) {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &PoseRecorder::poseCallback, this);

        file_.open("/tmp/car_positions.txt", std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            ROS_ERROR("Failed to open file for writing.");
            ros::shutdown();
        } else {
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

    /**
     * @brief Callback function to process received pose messages.
     * 
     * Records the pose to a file if the car has moved more than 10 meters
     * from the last recorded position.
     * 
     * @param msg Pointer to the received PoseStamped message
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        if (!first_msg_received_) {
            last_x_ = x;
            last_y_ = y;
            first_msg_received_ = true;
            recordPosition(x, y);
            ROS_INFO("Initial position recorded: (%f, %f)", x, y);
            return;
        }

        double dist = std::hypot(x - last_x_, y - last_y_);
        if (dist >= 10.0) {
            last_x_ = x;
            last_y_ = y;
            recordPosition(x, y);
            ROS_INFO("Moved 10m+, position recorded: (%f, %f)", x, y);
        }
    }

    /**
     * @brief Writes the current (x, y) position to the output file.
     * 
     * @param x X coordinate
     * @param y Y coordinate
     */
    void recordPosition(double x, double y) {
        if (file_.is_open()) {
            file_ << x << "," << y << "\n";
            file_.flush();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_recorder_node");
    PoseRecorder recorder;
    ros::spin();
    return 0;
}

