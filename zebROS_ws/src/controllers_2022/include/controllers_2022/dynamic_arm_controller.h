#ifndef DYNAMIC_ARM_CONTROLLER
#define DYNAMIC_ARM_CONTROLLER

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2022_msgs/DynamicArmSrv.h"
#include <dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h>
#include <controllers_2022/DynamicArmConfig.h>

namespace dynamic_arm_controller
{

  class DynamicArmCommand
  {
  public:
    DynamicArmCommand()
    {
      data_ = 0.0;
      go_slow_ = false; // only for motion magic
      use_percent_output_ = true;
    }
    DynamicArmCommand(double data, bool use_percent_output, bool go_slow)
    {
      data_ = data;
      go_slow_ = go_slow;
      use_percent_output_ = use_percent_output;
    }
    double GetData() const
    {
    return data_;
    }
    bool GetUsingPercentOutput() const
    {
    return use_percent_output_;
    }
    bool GetGoSlow() const
    {
    return go_slow_;
    }


  private:
    double data_;
    bool use_percent_output_;
    bool go_slow_;
  };

  class DynamicArmController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface>
  {
  public:
    DynamicArmController()
    {
    }

    virtual bool init(hardware_interface::RobotHW *hw,
                      ros::NodeHandle             &root_nh,
                      ros::NodeHandle             &controller_nh) override;
    virtual void starting(const ros::Time &time) override;
    virtual void update(const ros::Time & time, const ros::Duration& period) override;
    virtual void stopping(const ros::Time &time) override;

    bool cmdService(controllers_2022_msgs::DynamicArmSrv::Request &req,
                    controllers_2022_msgs::DynamicArmSrv::Response &res);

    // void callback(dynamic_arm_controller::DynamicArmConfig &config, uint32_t level); // This was in the 2019 elevator controller, not sure if we need it

  private:
    talon_controllers::TalonControllerInterface dynamic_arm_joint_; //interface for the talon joint

    realtime_tools::RealtimeBuffer<DynamicArmCommand> command_buffer_; // this is the buffer for ommands to be published
    ros::ServiceServer dynamic_arm_service_; //service for receiving commands

  bool zeroed_;
  bool last_zeroed_;

  DynamicReconfigureWrapper<DynamicArmConfig> dynamic_reconfigure_server_;
  DynamicArmConfig config_;

  ros::Time last_time_down_;
  };
}

#endif
