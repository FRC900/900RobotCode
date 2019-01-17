#ifndef INTAKE_CONTROLLER
#define INTAKE_CONTROLLER

#include <ros/ros.h>
#include <vector>
#include <hardware_interface/joint_state_interface.h> //other than talon data
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <boost/shared_ptr.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller.h> //for writing controllers
#include <atomic>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

namespace climber_controller
{
class ClimberController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            ClimberController()
            {
            }

            virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh);
            virtual void starting(const ros::Time &time);
            virtual void update(const ros::Time & time, const ros::Duration& period);
            virtual void stopping(const ros::Time &time);

            virtual bool activateSrv(std_srvs::SetBool::Request &req,
					                 std_srvs::SetBool::Response &/*res*/);

        private:
            std::vector<std::string> joint_names_; //still not used, but we might have to for config file things?
            hardware_interface::JointHandle climber_in_; //interface for the in/out solenoid joint

            realtime_tools::RealtimeBuffer<double> climber_in_cmd_; //buffer for in/out commands

            ros::ServiceServer service_command_; //service for receiving commands
}; //class

} //namespace
#endif
