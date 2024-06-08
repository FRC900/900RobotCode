#include <stdexcept>
#include <ros/time.h>
#include "auto_node/pause_function.h"
#include "auto_node/param_read.h"

PauseFunction::PauseFunction(const std::string &name, XmlRpc::XmlRpcValue action_data)
    : FunctionBase(name, true)
{
    // read duration - user could've entered a double or an int, we don't know which
    double duration_secs;
    if (!readFloatParam("duration", action_data, duration_secs))
    {
        throw std::invalid_argument("Auto function " + name + " - duration is not a double or int in pause action");
    }
    duration_ = ros::Duration(duration_secs);
}

bool PauseFunction::start(void)
{
    start_ = ros::Time::now();
    return true;
}

bool PauseFunction::is_finished(void) 
{
    return (ros::Time::now() - start_) >= duration_;
}