#include <stdexcept>
#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "auto_node/cmd_vel_function.h"
#include "auto_node/param_read.h"

CmdVelFunction::CmdVelFunction(ros::NodeHandle nh, const std::string &name, XmlRpc::XmlRpcValue action_data)
    : FunctionBase(name, true)
{
    // read duration - user could've entered a double or an int, we don't know which
    double duration_secs;
    if (!readFloatParam("duration", action_data, duration_secs))
    {
        throw std::invalid_argument("Auto function " + name + " - duration is not a double or int in pause action");
    }
    duration_ = ros::Duration(duration_secs);

    if (!action_data.hasMember("cmd_vel"))
    {
        throw std::invalid_argument("Auto function " + name + " missing 'cmd_vel' field");
    }
    XmlRpc::XmlRpcValue cmd_vel_data = action_data["cmd_vel"];

    if (!readFloatParam("x", cmd_vel_data, cmd_vel_.linear.x))
    {
        throw std::invalid_argument("Auto function " + name + " error reading cmd_vel 'x' field");
    }
    if (!readFloatParam("y", cmd_vel_data, cmd_vel_.linear.y))
    {
        throw std::invalid_argument("Auto function " + name + " error reading cmd_vel 'y' field");
    }
    orient_command_pub_ = nh.advertise<std_msgs::Float64>("/teleop/orientation_command", 1);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    current_yaw_sub_ = nh.subscribe("/teleop/orient_strafing/state", 1, &CmdVelFunction::yaw_callback, this, ros::TransportHints().tcpNoDelay());
    orientation_effort_sub_ = nh.subscribe("/teleop/orient_strafing/control_effort", 1, &CmdVelFunction::orientation_effort_callback, this, ros::TransportHints().tcpNoDelay());

    brake_srv_ = nh.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, {{"tcp_nodelay", "1"}});
}

bool CmdVelFunction::start(void)
{
    start_ = ros::Time::now();
    ROS_INFO_STREAM("Auto function cmd_vel x = " << cmd_vel_.linear.x <<
                    " y = " << cmd_vel_.linear.y <<
                    " for " << duration_ << " seconds");
    orient_msg_.data = current_yaw_; // Hold starting orientation while moving
    orient_command_pub_.publish(orient_msg_);
    return true;
}

bool CmdVelFunction::update(void)
{
    cmd_vel_.angular.z = current_orient_effort_;
    cmd_vel_pub_.publish(cmd_vel_);
    orient_command_pub_.publish(orient_msg_); // need to do this to keep control of rotation
    return true;
}

bool CmdVelFunction::finish(void)
{
    geometry_msgs::Twist stop_cmd_vel;
    stop_cmd_vel.linear.x = 0;
    stop_cmd_vel.linear.y = 0;
    stop_cmd_vel.linear.z = 0;
    stop_cmd_vel.angular.x = 0;
    stop_cmd_vel.angular.y = 0;
    stop_cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(stop_cmd_vel);
    if (std_srvs::Empty empty; !brake_srv_.call(empty))
    {
        ROS_ERROR_STREAM("BrakeSrv call failed in auto cmd_vel step " << get_name());
        return false;
    }
    ROS_INFO_STREAM("Auto action " << get_name() << " finished");
    return true;
}

bool CmdVelFunction::is_finished(void) 
{
    return (ros::Time::now() - start_) >= duration_;
}

void CmdVelFunction::yaw_callback(const std_msgs::Float64 &msg)
{
    current_yaw_ = msg.data;
}
void CmdVelFunction::orientation_effort_callback(const std_msgs::Float64 &msg)
{
    current_orient_effort_ = msg.data;
}