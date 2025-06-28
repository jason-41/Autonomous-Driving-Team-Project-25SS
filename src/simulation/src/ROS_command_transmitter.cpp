#include <math.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "simulation/VehicleControl.h"

#include "libsocket/inetclientdgram.hpp"
#include "libsocket/exception.hpp"


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

float current_command[4];

class UDPPoseStreamer
{
public:
  UDPPoseStreamer(const std::string &udp_address,
                  const std::string &udp_port,
                const std::string &node_name)
      : dgram_client(LIBSOCKET_IPv4),
        ip_address(udp_address),
        port(udp_port),
        node_name(node_name)
  {
  }

  virtual ~UDPPoseStreamer() { dgram_client.destroy(); }

  // private:
  void send_command_to_Unity(float command_arr[])
  {
    float temp_arr[4] = {
        static_cast<float>(command_arr[0]),
        static_cast<float>(command_arr[1]),
        static_cast<float>(command_arr[2]),
        static_cast<float>(command_arr[3]),
    };

    bool accept = true;
    for (uint i = 0; i < 4; i++)
    {
      if (std::isnan(temp_arr[i]))
      {
        accept = false;
        break;
      }
    }

    if (accept)
    {
      static const size_t int32_size = sizeof(uint32_t);
      static const size_t pose_size = sizeof(float) * 4;
      static const size_t packet_size = pose_size;
      uint8_t packet_data[packet_size];
      // ROS_INFO("Sending packet: %.2f, %.2f, %.2f", temp_arr[0], temp_arr[1], temp_arr[2]);
      memcpy(packet_data, &temp_arr, pose_size);
      dgram_client.sndto(&packet_data, packet_size, ip_address, port);
    }
    else
    {
      ROS_WARN("[%s] Received NaN in command message.", this->node_name.c_str());
    }
  }

  libsocket::inet_dgram_client dgram_client;
  std::string node_name;
  std::string ip_address;
  std::string port;
};

// void command_callback(const mav_msgs::Actuators &cmd)
// {
//   current_command[0] = cmd.angular_velocities[0];
//   current_command[1] = cmd.angular_velocities[1];
//   current_command[2] = cmd.angular_velocities[2];
//   current_command[3] = cmd.angular_velocities[3];
//   return;
// }


void command_callback(const simulation::VehicleControl &cmd)
{
  current_command[0] = cmd.Throttle;
  current_command[1] = cmd.Steering;
  current_command[2] = cmd.Brake;
  current_command[3] = cmd.Reserved;
  return;
}
int main(int argc, char **argv)
{
  
  const std::string command_topic("car_command");
  ros::init(argc, argv, "ROS_Unity_command_Tx");
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getName();

  ros::Rate loop_rate(100);

  std::string ip_address;
  int port;

  ROS_INFO("[%s] Waiting for Tx parameters to be available...", node_name.c_str());
  wait_for_param<std::string>("/controlConfig/address", ip_address, "127.0.0.1");
  wait_for_param<int>("/controlConfig/port", port, 6657);

  ROS_INFO("[%s] Starting command transmit to Unity via UDP port %s:%d", node_name.c_str(), ip_address.c_str(), port);
  ROS_INFO("[%s] Listening to command topic: [%s] ", node_name.c_str(), command_topic.c_str());
  UDPPoseStreamer streamer(ip_address, std::to_string(port), node_name);
  ros::Subscriber sub = n.subscribe(command_topic, 1, command_callback);

  while (ros::ok())
  {
    streamer.send_command_to_Unity(current_command);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
