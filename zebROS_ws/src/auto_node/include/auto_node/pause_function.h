#ifndef PAUSE_FUNCTION_INC__
#define PAUSE_FUNCTION_INC__

#include "auto_node/function_base.h"

class PauseFunction : public FunctionBase
{
public:
    PauseFunction(const std::string &name,
                  XmlRpc::XmlRpcValue action_data);
    bool start(void) override;
    bool is_finished(void) override;
private:
    ros::Duration duration_;
    ros::Time start_;
};

#endif
