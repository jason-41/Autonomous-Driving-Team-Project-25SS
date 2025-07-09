#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include "math.h"
constexpr float loop_interval = 0.05;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_controller_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<simulation::VehicleControl>("car_command", 1);
    ros::Rate loop_rate(1 / loop_interval);
    float elapsed_time = 0.0f;

    while (ros::ok())
    {
        simulation::VehicleControl msg;
        msg.Throttle = 0.5f; // Throttle value from -1 to 1, this is the torque applied to the motors
        msg.Steering =  sin(6.28 * elapsed_time) * 0.5; //Steering value from -1 to 1, in which: positive value <=> turning right
        msg.Brake = 0.0f; // Brake value from 0 to 1, this will apply brake torque to stop the car
        msg.Reserved = 0.0f; // Not used!

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        elapsed_time += loop_interval;

    }

    return 0;
}