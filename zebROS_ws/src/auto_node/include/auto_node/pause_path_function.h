#ifndef PAUSE_PATH_FUNCTION_INC__
#define PAUSE_PATH_FUNCTION_INC__

#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "auto_node/function_base.h"

class PausePathFunction : public FunctionBase
{
public:
    PausePathFunction(const ros::NodeHandle &nh,
                      const std::string &name,
                      XmlRpc::XmlRpcValue action_data);
    bool start(void) override;
    bool is_finished(void) override;
private:
    bool pause_;
    ros::NodeHandle nh_;
    ros::ServiceClient pause_srv_;
};

#endif
