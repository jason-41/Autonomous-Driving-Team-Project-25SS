#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <fstream>
#include "json_nlohmann/json.hpp"

using json = nlohmann::json;

const std::string config_file_path("/unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json");

int main(int argc, char** argv)
{
    ros::init(argc, argv, "json_param_loader");
    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();

    std::string config_path = ros::package::getPath("simulation") + config_file_path;
    std::ifstream config_file(config_path);
    if (!config_file.is_open())
    {
        ROS_ERROR("[%s] Cannot open JSON config at %s", node_name.c_str(), config_path.c_str());
        return -1;
    }

    json config;
    try
    {
        config_file >> config;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("[%s] %s", node_name.c_str(), e.what());
        ROS_ERROR("[%s] JSON reader corrupts. Is the configuration file correctly structured?", node_name.c_str());
        return -1;
    }
    std::string ctrl_addr;
    int ctrl_port;
    try
    {
        ctrl_addr = config["controlConfig"]["address"];
        ctrl_port = config["controlConfig"]["port"];
        ros::param::set("/controlConfig/address", ctrl_addr);
        ros::param::set("/controlConfig/port", ctrl_port);
        ROS_INFO("[%s] Read control message Tx address: %s, port: %d", node_name.c_str(), ctrl_addr.c_str(), ctrl_port);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("[%s] %s", node_name.c_str(), e.what());
        ROS_ERROR("[%s] Field \"controlConfig\" in JSON config file \"%s\" is incomplete or of wrong data type. Is the field correctly structured as in example?", node_name.c_str(), config_path.c_str());
    }
    
    std::string sensor_addr;
    int sensor_port;
    try
    {
        sensor_addr = config["sensorConfig"]["address"];
        sensor_port = config["sensorConfig"]["port"];
        ros::param::set("/sensorConfig/address", sensor_addr);
        ros::param::set("/sensorConfig/port", sensor_port);
        ROS_INFO("[%s] Read sensor message Rx address: %s, port: %d", node_name.c_str(), sensor_addr.c_str(), sensor_port);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("[%s] %s", node_name.c_str(), e.what());
        ROS_ERROR("[%s] Field \"sensorConfig\" in JSON config file \"%s\" is incomplete or of wrong data type. Is the field correctly structured as in example?", node_name.c_str(), config_path.c_str());
    }

    return 0;
}