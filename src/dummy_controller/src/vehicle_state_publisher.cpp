#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vehicle_state_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("vehicle_state", 10);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // 设置正确的目标坐标系
    std::string target_frame = "OurCar/INS";
    ros::Rate rate(50);
    
    while (ros::ok()) {
        geometry_msgs::Pose msg;
        try {
            auto transform = tf_buffer.lookupTransform("world", target_frame, ros::Time(0));
            msg.position.x = transform.transform.translation.x;
            msg.position.y = transform.transform.translation.y;
            msg.position.z = transform.transform.translation.z;
            msg.orientation = transform.transform.rotation;
            pub.publish(msg);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF转换失败: %s", ex.what());
            // 如果持续失败，可以添加短暂延迟
            ros::Duration(0.1).sleep();
        }
        rate.sleep();
    }
    return 0;
}
