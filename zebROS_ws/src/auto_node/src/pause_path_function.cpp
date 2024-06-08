#include <stdexcept>
#include "auto_node/pause_path_function.h"
#include "auto_node/param_read.h"

#include "std_srvs/SetBool.h"

PausePathFunction::PausePathFunction(const ros::NodeHandle &nh, const std::string &name, XmlRpc::XmlRpcValue action_data)
    : FunctionBase(name, true)
{
    // read duration - user could've entered a double or an int, we don't know which
    if (!readBoolParam("pause", action_data, pause_))
    {
        throw std::invalid_argument("Auto function " + name + " - pause is not a bool param in pause path action");
    }
    pause_srv_ = nh_.serviceClient<std_srvs::SetBool>("/path_follower/pause_path", false, {{"tcp_nodelay", "1"}});
}

bool PausePathFunction::start(void)
{
    std_srvs::SetBool srv;
    srv.request.data = pause_;
    if (!pause_srv_.call(srv))
    {
        ROS_ERROR_STREAM("Auto node - pause path service call failed");
        return false;
    }
    ROS_INFO_STREAM("Auto node - pause path service call succeeded");
    return true;
}

bool PausePathFunction::is_finished(void) 
{
    return true;
}