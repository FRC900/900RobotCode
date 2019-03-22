#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_msgs/Bool.h>
#include "panel_intake_controller/PanelIntakeSrv.h"

namespace panel_intake_controller
{

//define class to hold command data - so we only need one realtime buffer
class PanelCommand
{
	public:
		// Set default state of mechanism here
		PanelCommand()
			: claw_cmd_(false)
			, push_cmd_(false)
		{
		}
		PanelCommand(bool claw_cmd, bool push_cmd)
		{
			claw_cmd_ = claw_cmd;
			push_cmd_ = push_cmd;
		}
		bool claw_cmd_;
		bool push_cmd_;
};

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class PanelIntakeController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            PanelIntakeController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

            virtual bool cmdService(panel_intake_controller::PanelIntakeSrv::Request &req,
					                panel_intake_controller::PanelIntakeSrv::Response &res);

        private:
            hardware_interface::JointHandle claw_joint_; //interface for the in/out solenoid joint
			hardware_interface::JointHandle push_joint_;

			realtime_tools::RealtimeBuffer<PanelCommand> panel_cmd_; //buffer for clamp and extend commands

            ros::ServiceServer panel_intake_service_; //service for receiving commands
}; //class

} //namespace
