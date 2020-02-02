#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include <pluginlib/class_list_macros.h> //to compile as a controller

#include <controllers_2020_msgs/ControlPanelSrv.h>
#include <talon_state_msgs/TalonState.h>

namespace control_panel_controller
{

	//this is the controller class, used to make a controller


	class ControlPanelCommand
	{
		public:
			ControlPanelCommand()
				: set_point_(0.0)
				  , panel_arm_extend_(false)
		{
		}
			ControlPanelCommand(double set_point, bool panel_arm_extend)
			{
				set_point_ = set_point;
				panel_arm_extend_ = panel_arm_extend;
			}
			double set_point_;
			bool panel_arm_extend_;
	};




	class ControlPanelController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
	{
		public:
			ControlPanelController()
			{
			}

			//the four essential functions for a controller: init, starting, update, stopping

			virtual bool init(hardware_interface::RobotHW *hw,
					ros::NodeHandle             &root_nh,
					ros::NodeHandle             &controller_nh) override;
			virtual void starting(const ros::Time &time) override;
			virtual void update(const ros::Time & time, const ros::Duration& period) override;
			virtual void stopping(const ros::Time &time) override;
			bool cmdService (controllers_2020_msgs::ControlPanelSrv::Request &req, controllers_2020_msgs::ControlPanelSrv::Response &/*response*/);
			void talonStateCallback(const talon_state_msgs::TalonState &talon_state);

		private:
			talon_controllers::TalonMotionMagicCloseLoopControllerInterface control_panel_joint_;//interface for the control panel turning motor
			hardware_interface::JointHandle control_panel_arm_joint_; //interface for the control panel arm solenoid
			realtime_tools::RealtimeBuffer<ControlPanelCommand> control_panel_cmd_;
			ros::ServiceServer control_panel_service_;
			double control_panel_diameter_;
			double wheel_diameter_;

			ros::Subscriber talon_state_sub_;
			double cur_motor_position_;

	}; //class

} //namespace

