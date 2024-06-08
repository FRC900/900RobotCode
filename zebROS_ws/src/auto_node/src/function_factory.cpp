#include <memory>
#include <stdexcept>

#include "ros/node_handle.h"
#include "auto_node/cmd_vel_function.h"
#include "auto_node/pause_function.h"
#include "auto_node/pause_path_function.h"

static std::map<std::string, std::unique_ptr<FunctionBase>> function_cache;

FunctionBase *create_function(ros::NodeHandle &nh,
                              const std::string &type,
                              const std::string &name,
                              XmlRpc::XmlRpcValue action_data)
{
    // Reuse previous instances of functions to prevent having to constantly
    // create and destroy serviceclients, action clients, etc.
    if (auto it = function_cache.find(name);
        it != function_cache.cend())
    {
        return it->second.get();
    }

    if (type == "cmd_vel")
    {
        auto [it, _] = function_cache.try_emplace(name, std::make_unique<CmdVelFunction>(nh, name, action_data));
        return it->second.get();
    }

    if (type == "pause")
    {
        auto [it, _] = function_cache.try_emplace(name, std::make_unique<PauseFunction>(name, action_data));
        return it->second.get();
    }

    if (type == "pause_path")
    {
        auto [it, _] = function_cache.try_emplace(name, std::make_unique<PausePathFunction>(nh, name, action_data));
        return it->second.get();
    }   
    throw std::invalid_argument("Unknown function type: " + type);
    return nullptr;
}