#include <vector>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "unity_stream_parser.h"
#include "rgb_camera_parser.h"
#include "depth_camera_parser.h"
#include "imu_parser.h"
#include "true_state_parser.h"


template<typename T>
bool wait_for_param(const std::string& key, T& value, const T& default_val, double wait_duration = 5.0, double check_interval = 0.1)
{
    ros::Time start_time = ros::Time::now();
    ros::Duration interval(check_interval);

    while (ros::ok())
    {
        if (ros::param::get(key, value))
        {
            ROS_INFO_STREAM("{Parameter Inquiry} [" << key << "]: " << value);
            return true;
        }

        if ((ros::Time::now() - start_time).toSec() > wait_duration)
        {
            ROS_ERROR_STREAM("{Parameter Inquiry} ROS Parameter [" << key << "] not found after " << wait_duration << "s, using default value: " << default_val);
            value = default_val;
            return false;
        }

        interval.sleep();
    }
    return false;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Unity_ROS_message_Rx");
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getName();

  int port;
  ROS_INFO("[%s] Waiting for Rx parameters to be available...", node_name.c_str());

  wait_for_param<int>("/sensorConfig/port", port, 9998);

  TCPStreamReader *stream_reader;
  try
  {
    stream_reader = new TCPStreamReader("0.0.0.0", std::to_string(port));
  }
  catch (libsocket::socket_exception &e)
  {
    ROS_INFO("[%s] %s", node_name.c_str(), e.mesg.c_str());
    return 0;
  }

  ROS_INFO("[%s] Waiting for TCP connection on port %d", node_name.c_str(), port);
  stream_reader->WaitConnect();
  ROS_INFO("[%s] Got a connection!", node_name.c_str());

  std::vector<std::shared_ptr<UnityStreamParser>> stream_parsers(UnityMessageType::MESSAGE_TYPE_COUNT);
  stream_parsers[UnityMessageType::UNITY_STATE] = std::make_shared<TrueStateParser>();
  stream_parsers[UnityMessageType::UNITY_IMU] = std::make_shared<IMUParser>();
  stream_parsers[UnityMessageType::UNITY_CAMERA] = std::make_shared<RGBCameraParser>();
  stream_parsers[UnityMessageType::UNITY_DEPTH] = std::make_shared<DepthCameraParser>();


  bool first_message_received = false;
  int64_t time_offset_ns = 0;

  while (stream_reader->Good() && ros::ok())
  {
    uint32_t magic = stream_reader->ReadUInt();
    if (magic == 0xDEADC0DE)
    {
      int64_t ros_time_now_ns = static_cast<int64_t>(ros::Time::now().toSec() * 1e7);
      UnityHeader header;
      header.type = static_cast<UnityMessageType>(stream_reader->ReadUInt());
      int64_t timestamp_ns = static_cast<int64_t>(stream_reader->ReadUInt64());
      header.name = stream_reader->ReadString();

      if(!first_message_received)
      {
        time_offset_ns = ros_time_now_ns - timestamp_ns;
        first_message_received = true;
        ROS_INFO("[%s] Waiting for connection...", node_name.c_str());
        ROS_INFO_STREAM(node_name << "Unity-ROS timeSync: time_offset = " << time_offset_ns << "ns");
      }
      header.timestamp = static_cast<double>(timestamp_ns * 1e-7);
      if (header.type < UnityMessageType::MESSAGE_TYPE_COUNT)
      {
        stream_parsers[header.type]->ParseMessage(header, *stream_reader, time_offset_ns * 1e-7);
      }
      else
      {
        ROS_ERROR("[%s] Receives message of unknown datatype of %d", node_name.c_str() ,static_cast<unsigned int>(header.type));
      }
    }
    else
    {
      ROS_ERROR("[%s] Stream corrupted, could not parse unity message", node_name.c_str());
    }

    ros::spinOnce();
  }

  return 0;
}
