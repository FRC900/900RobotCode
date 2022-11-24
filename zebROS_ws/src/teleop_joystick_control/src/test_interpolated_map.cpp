#include "teleop_joystick_control/interpolating_map.h"
#include "teleop_joystick_control/store_xy.h"
#include "ros/ros.h"
#include <string>
#include "std_msgs/Float64.h"


wpi::interpolating_map<double, store_xy> joystick_values;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_interpolated_map");

    ros::NodeHandle n;
    
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/test_interpolated_map_pub", 10);

    ros::Rate rate(10)

    while (ros::ok())
    {
        std_msgs::Float64 msg;

        joystick_values.insert(atan2(-0.5, -1.0), store_xy(-1.0, -0.5));
        joystick_values.insert(atan2(0.5, -1.0), store_xy(-1.0, 0.5));
        joystick_values.insert(atan2(-0.5, 1.0), store_xy(1.0, -0.5));
        joystick_values.insert(atan2(0.5, 1.0), store_xy(1.0, 0.5));
        joystick_values.insert(atan2(-1.0, -0.5), store_xy(-0.5, -1.0));
        joystick_values.insert(atan2(1.0, -0.5), store_xy(-0.5, 1.0));
        joystick_values.insert(atan2(-1.0, 0.5), store_xy(0.5, -1.0));
        joystick_values.insert(atan2(1.0, 0.5), store_xy(0.5, 1.0));

        msg.data = joystick_values[atan2(1.0, 0.5)].hypot();
    }
 
    return 0;
}
