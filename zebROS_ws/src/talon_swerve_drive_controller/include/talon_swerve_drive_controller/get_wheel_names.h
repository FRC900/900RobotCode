#ifndef GET_WHEEL_NAMES__
#define GET_WHEEL_NAMES__

#include <array>
#include <string>
#include <ros/node_handle.h>

template <size_t WHEELCOUNT>
bool getWheelNames(const ros::NodeHandle &controller_nh,
                   const std::string &controller_name,
                   const std::string &wheel_param,
                   std::array<std::string, WHEELCOUNT> &wheel_names)
{
    XmlRpc::XmlRpcValue wheel_list;
    if (!controller_nh.getParam(wheel_param, wheel_list))
    {
        ROS_ERROR_STREAM_NAMED(controller_name,
                               "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
    }

    if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        if (wheel_list.size() == 0)
        {
            ROS_ERROR_STREAM_NAMED(controller_name,
                                   "Wheel param '" << wheel_param << "' is an empty list");
            return false;
        }
        if (wheel_list.size() != WHEELCOUNT)
        {
            ROS_ERROR_STREAM_NAMED(controller_name,
                                   "Wheel param size (" << wheel_list.size() << " != WHEELCOUNT (" << WHEELCOUNT << ").");
            return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
            if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR_STREAM_NAMED(controller_name,
                                       "Wheel param '" << wheel_param << "' #" << i << " isn't a string.");
                return false;
            }
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
            wheel_names[i] = static_cast<std::string>(wheel_list[i]);
        }
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(controller_name,
                               "Wheel param '" << wheel_param << "' is neither a list of strings nor a string.");
        return false;
    }

    return true;
}
#endif