#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h>


#include <controllers_2020_msgs/ControlPanelSrv.h>

namespace control_panel_controller
{

	//this is the controller class, used to make a controller


	class ControlPanelCommand
	{
		public:
			ControlPanelCommand()
				: set_point_(0.0)
		{
		}
			ControlPanelCommand(double set_point)
			{
				set_point_ = set_point;
			}
			double set_point_;
	};




	class ControlPanelController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface>
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

		private:
			talon_controllers::TalonMotionMagicCloseLoopControllerInterface control_panel_joint_;//interface for the control panel turning motor
			realtime_tools::RealtimeBuffer<ControlPanelCommand> control_panel_cmd_;
			ros::ServiceServer control_panel_service_;
			double control_panel_diameter_;
			double wheel_diameter_;

			double cur_motor_position_;

	}; //class

} //namespace

