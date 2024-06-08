#ifndef CMD_VEL_FUNCTION_INC__
#define CMD_VEL_FUNCTION_INC__
#include "ros/node_handle.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include "auto_node/function_base.h"
class CmdVelFunction : public FunctionBase
{
public:
    CmdVelFunction(ros::NodeHandle nh,
                   const std::string &name,
                   XmlRpc::XmlRpcValue action_data);
    bool start(void) override;
    bool finish(void) override;
    bool update(void) override;
    bool is_finished(void) override;
private:
    ros::Duration duration_;
    ros::Time start_;
    geometry_msgs::Twist cmd_vel_;
    std_msgs::Float64 orient_msg_;
    ros::Publisher orient_command_pub_;
    ros::Publisher cmd_vel_pub_;

    ros::Subscriber current_yaw_sub_;
    void yaw_callback(const std_msgs::Float64 &msg);
    double current_yaw_;

    ros::Subscriber orientation_effort_sub_;
    void orientation_effort_callback(const std_msgs::Float64 &msg);
    double current_orient_effort_{0};

    ros::ServiceClient brake_srv_;
};

#endif
