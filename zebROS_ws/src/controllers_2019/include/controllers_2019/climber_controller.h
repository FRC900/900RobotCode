#ifndef CLIMBER_CONTROLLER
#define CLIMBER_CONTROLLER

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_srvs/SetBool.h>

namespace climber_controller_2019 //year necessary to differentiate from 2020 one
{
class ClimberController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            ClimberController()
            {
            }

            virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

            bool activateSrv(std_srvs::SetBool::Request &req,
							 std_srvs::SetBool::Response &/*res*/,
							 realtime_tools::RealtimeBuffer<bool> &realtime_buffer);

        private:
            std::vector<std::string> joint_names_; //still not used, but we might have to for config file things?
            hardware_interface::JointHandle feet_retract_; //interface for the in/out solenoid joint
            hardware_interface::JointHandle ski_retract_; //interface for the in/out solenoid joint

            realtime_tools::RealtimeBuffer<bool> feet_retract_cmd_; //buffer for in/out commands

            ros::ServiceServer feet_retract_service_; //service for receiving commands

            hardware_interface::JointHandle release_endgame_; //interface for the in/out solenoid joint

            realtime_tools::RealtimeBuffer<bool> release_endgame_cmd_; //buffer for in/out commands

            ros::ServiceServer release_endgame_service_; //service for receiving commands

			ros::Subscriber navX_sub_;
}; //class

} //namespace
#endif
