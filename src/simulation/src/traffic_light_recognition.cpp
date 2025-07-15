#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>

// 找到语义图中黄色像素，返回最靠近图像中心的点
bool findCenterClosestTrafficLight(const cv::Mat& semantic_img, cv::Point& center)
{
    cv::Mat mask;
    cv::inRange(semantic_img, cv::Scalar(0, 200, 200), cv::Scalar(50, 255, 255), mask);

    double min_dist = std::numeric_limits<double>::max();
    bool found = false;
    cv::Point img_center(semantic_img.cols / 2, semantic_img.rows / 2);

    for (int y = 0; y < mask.rows; y++)
    {
        for (int x = 0; x < mask.cols; x++)
        {
            if (mask.at<uchar>(y, x) > 0)
            {
                double dist = std::abs(x - img_center.x);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    center = cv::Point(x, y);
                    found = true;
                }
            }
        }
    }
    return found;
}

// 简单判定红绿灯颜色
std::string decideTrafficLightState(const cv::Mat& roi)
{
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red_mask1, red_mask2, red_mask;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    double red_ratio = cv::countNonZero(red_mask) / double(roi.total());

    ROS_INFO("Red ratio: %.3f", red_ratio);
    if (red_ratio > 0.015)
        return "RED";
    else
        return "UNKNOWN";
}

class TrafficLightRecognizer
{
public:
    TrafficLightRecognizer(ros::NodeHandle& nh) : it_(nh), tf_listener_(tf_buffer_)
    {
        image_transport::TransportHints hints("raw", ros::TransportHints(), ros::NodeHandle("~"));
        semantic_img_sub_ = it_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1, &TrafficLightRecognizer::semanticImgCb, this, hints);
        semantic_info_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/camera_info", 1, &TrafficLightRecognizer::semanticInfoCb, this);
        depth_img_sub_ = it_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1, &TrafficLightRecognizer::depthImgCb, this, hints);
        depth_info_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info", 1, &TrafficLightRecognizer::depthInfoCb, this);
        rgb_img_sub_ = it_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1, &TrafficLightRecognizer::rgbImgCb, this, hints);
        rgb_info_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/camera_info", 1, &TrafficLightRecognizer::rgbInfoCb, this);

        traffic_light_img_pub_ = it_.advertise("/traffic_light/image", 1);
        traffic_light_state_pub_ = nh.advertise<std_msgs::String>("/traffic_state", 1);
    }

    void semanticInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) { semantic_cam_info_ = *msg; }
    void depthInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) { depth_cam_info_ = *msg; }
    void rgbInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) { rgb_cam_info_ = *msg; }

    void semanticImgCb(const sensor_msgs::ImageConstPtr& msg) {
        try { semantic_img_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone(); } catch (...) { ROS_WARN("Failed semantic img"); }
    }
    void depthImgCb(const sensor_msgs::ImageConstPtr& msg) {
        try { depth_img_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone(); } catch (...) { ROS_WARN("Failed depth img"); }
    }
    void rgbImgCb(const sensor_msgs::ImageConstPtr& msg) {
        try { rgb_img_ = cv_bridge::toCvShare(msg, "bgr8")->image; } catch (...) { ROS_WARN("Failed rgb img"); }
    }
    
    double getVehicleYaw()
   {
       try
       {
         geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform("world", "OurCar/Center", ros::Time(0));
         tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
         double roll, pitch, yaw;
         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
         return yaw;
        }
       catch (tf2::TransformException &ex)
       {
         ROS_WARN("Failed to get vehicle yaw: %s", ex.what());
         return 0.0; 
        }
    }

    float estimateFakeDepthByArea()
   {
        if (semantic_img_.empty())
        return 8.0f; 

        cv::Mat mask;
        cv::inRange(semantic_img_, cv::Scalar(0, 200, 200), cv::Scalar(50, 255, 255), mask);

        double area = cv::countNonZero(mask);

        if (area < 1.0)
         return 8.0f;

        
        const float k = 50000.0f;
        float estimated_distance = sqrt(k / area);

        
        if (estimated_distance < 3.0f)
            {estimated_distance = 3.0f;}
        else if (estimated_distance > 20.0f)
            {estimated_distance = 20.0f;}

        //ROS_INFO("Area-based estimated fake depth: %.2f m (area=%.0f)", estimated_distance, area);
        return estimated_distance;
    }


    void process()
    {
        frame_count_++;
        if (frame_count_ % process_interval_ != 0)
            return;
        if (semantic_img_.empty() || depth_img_.empty() || rgb_img_.empty()) return;
        if (semantic_cam_info_.K.empty() || depth_cam_info_.K.empty() || rgb_cam_info_.K.empty()) return;

        cv::Point tl_center;
        if (!findCenterClosestTrafficLight(semantic_img_, tl_center))
        {
            ROS_INFO("No traffic light found");
            // 发布一张全黑图像代替红绿灯图像，清除上一次的图像
            cv::Mat black_image = cv::Mat::zeros(20, 20, CV_8UC3);  // 可以设成合适大小
            sensor_msgs::ImagePtr empty_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", black_image).toImageMsg();
            traffic_light_img_pub_.publish(empty_msg);

            // 发布 UNKNOWN 状态
            std_msgs::String state_msg;
            state_msg.data = "UNKNOWN";
            traffic_light_state_pub_.publish(state_msg);
            return;
        }

        float depth_val = depth_img_.at<float>(tl_center.y, tl_center.x);
        bool valid_depth_found = (depth_val > 0.1f && !std::isnan(depth_val));

        if (!valid_depth_found)
        {
            const int window_size = 2;
            for (int dy = -window_size; dy <= window_size; ++dy)
            {
                for (int dx = -window_size; dx <= window_size; ++dx)
                {
                    int ny = tl_center.y + dy;
                    int nx = tl_center.x + dx;
                    if (ny < 0 || ny >= depth_img_.rows || nx < 0 || nx >= depth_img_.cols) continue;

                    float neighbor_depth = depth_img_.at<float>(ny, nx);
                    if (neighbor_depth > 0.1f && !std::isnan(neighbor_depth))
                    {
                        depth_val = neighbor_depth;
                        tl_center = cv::Point(nx, ny);
                        valid_depth_found = true;
                        break;
                    }
                }
                if (valid_depth_found) break;
            }
        }

        if (valid_depth_found)
        {
            //ROS_INFO("Depth (corrected): %.3f", depth_val);
            cv::Point3d pt3d = pixelTo3D(tl_center.x, tl_center.y, depth_val, semantic_cam_info_);

            geometry_msgs::PointStamped pt_in, pt_out;
            pt_in.header.frame_id = semantic_cam_info_.header.frame_id;
            pt_in.header.stamp = ros::Time(0);
            pt_in.point.x = pt3d.x;
            pt_in.point.y = pt3d.y;
            pt_in.point.z = pt3d.z;

            try { pt_out = tf_buffer_.transform(pt_in, rgb_cam_info_.header.frame_id, ros::Duration(0.1)); }
            catch (tf2::TransformException& ex) { ROS_WARN("%s", ex.what()); return; }

            cv::Point2d uv_rgb = project3DToPixel(pt_out, rgb_cam_info_);
            extractAndPublishROI(uv_rgb);
        }
        else
        {
            ROS_WARN("No valid depth nearby, fallback to rough estimate");

            // === 1. 获取车辆yaw角 ===
            double yaw = getVehicleYaw(); 

            // === 2. 动态调整参数 ===
            float base_fake_depth = estimateFakeDepthByArea();
            float base_tx = t_x_;
            float base_ty = t_y_;
            float base_tz = t_z_;

            double yaw_abs = std::abs(yaw);
            float fake_depth = base_fake_depth + yaw_abs * 2.0f;
            float dynamic_tx = base_tx + yaw * 0.9f; 

            
            cv::Point3d pt3d = pixelTo3D(tl_center.x, tl_center.y, fake_depth, semantic_cam_info_);

            pt3d.x += dynamic_tx;
            pt3d.y += base_ty;
            pt3d.z += base_tz;

            double fx = rgb_cam_info_.K[0], fy = rgb_cam_info_.K[4], cx = rgb_cam_info_.K[2], cy = rgb_cam_info_.K[5];
            double u_rgb = fx * pt3d.x / pt3d.z + cx;
            double v_rgb = fy * pt3d.y / pt3d.z + cy;

            cv::Point2d uv_rgb(u_rgb, v_rgb);
            extractAndPublishROI(uv_rgb);
        }
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber semantic_img_sub_, depth_img_sub_, rgb_img_sub_;
    ros::Subscriber semantic_info_sub_, depth_info_sub_, rgb_info_sub_;
    image_transport::Publisher traffic_light_img_pub_;
    ros::Publisher traffic_light_state_pub_;

    sensor_msgs::CameraInfo semantic_cam_info_, depth_cam_info_, rgb_cam_info_;
    cv::Mat semantic_img_, depth_img_, rgb_img_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int frame_count_ = 0;
    int process_interval_ = 3;

    double t_x_ = 1.0;  
    double t_y_ = 0.0;
    double t_z_ = 0.0;

    cv::Point3d pixelTo3D(int u, int v, float depth, const sensor_msgs::CameraInfo& cam_info)
    {
        double fx = cam_info.K[0], fy = cam_info.K[4], cx = cam_info.K[2], cy = cam_info.K[5];
        double x = (u - cx) * depth / fx;
        double y = (v - cy) * depth / fy;
        return cv::Point3d(x, y, depth);
    }

    cv::Point2d project3DToPixel(const geometry_msgs::PointStamped& pt, const sensor_msgs::CameraInfo& cam_info)
    {
        double fx = cam_info.K[0], fy = cam_info.K[4], cx = cam_info.K[2], cy = cam_info.K[5];
        double u = fx * pt.point.x / pt.point.z + cx;
        double v = fy * pt.point.y / pt.point.z + cy;
        return cv::Point2d(u, v);
    }

    void extractAndPublishROI(cv::Point2d uv_rgb)
    {
        uv_rgb.x += 5;
        uv_rgb.y += 5;

        int roi_width = 15;
        int roi_height = 20;
        int x = std::max(0, int(uv_rgb.x) - roi_width / 2);
        int y = std::max(0, int(uv_rgb.y) - roi_height / 2);
        int w = std::min(roi_width, rgb_img_.cols - x);
        int h = std::min(roi_height, rgb_img_.rows - y);
        if (w <= 0 || h <= 0) return;

        cv::Mat roi = rgb_img_(cv::Rect(x, y, w, h));
        sensor_msgs::ImagePtr roi_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg();
        traffic_light_img_pub_.publish(roi_msg);

        std_msgs::String state_msg;
        state_msg.data = decideTrafficLightState(roi);
        traffic_light_state_pub_.publish(state_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light_recognition_node");
    ros::NodeHandle nh;
    TrafficLightRecognizer recognizer(nh);
    ros::Rate rate(7);

    while (ros::ok())
    {
        ros::spinOnce();
        recognizer.process();
        rate.sleep();
    }
    return 0;
}
