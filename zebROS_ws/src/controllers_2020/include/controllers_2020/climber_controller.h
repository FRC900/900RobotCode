#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include <pluginlib/class_list_macros.h> //to compile as a controller

//REMEMBER TO INCLUDE CUSTOM SERVICE
#include <controllers_2020_msgs/ClimberSrv.h>

namespace climber_controller_2020
{


//class to store command data
class ClimberCommand
{
	public:
		ClimberCommand()
			: winch_set_point_(0.0),
			  deploy_(false),
			  brake_(false)
		{}
		ClimberCommand(double winch_set_point, bool deploy, bool brake)
		{
			winch_set_point_ = winch_set_point;
			deploy_ = deploy;
			brake_ = brake;
		}
		double winch_set_point_;
		bool deploy_;
		bool brake_;
};

//this is the controller class, used to make a controller
class ClimberController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{
        public:
            ClimberController()
            {
            }

			//the four essential functions for a controller: init, starting, update, stopping

			virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

			bool cmdService(controllers_2020_msgs::ClimberSrv::Request &req,
							controllers_2020_msgs::ClimberSrv::Response &res);
        private:
			talon_controllers::TalonMotionMagicCloseLoopControllerInterface winch_joint_; //TODO correct type?
			hardware_interface::JointHandle deploy_joint_; //piston to deploy the climber
			hardware_interface::JointHandle brake_joint_; //piston brake

			ros::ServiceServer climber_service_;
			realtime_tools::RealtimeBuffer<ClimberCommand> cmd_buffer_;

			double softlimit_offset_;

}; //class

} //namespace

