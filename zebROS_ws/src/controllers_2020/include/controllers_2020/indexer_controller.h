#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include <pluginlib/class_list_macros.h> //to compile as a controller

//REMEMBER TO INCLUDE CUSTOM SERVICE

#include <controllers_2020_msgs/IndexerSrv.h>

namespace indexer_controller
{

//this is the controller class, used to make a controller
class IndexerCommand
{
	public:
		IndexerCommand()
			: indexer_forward_(false)
			, indexer_reverse_(false)
		{
		}
		IndexerCommand(bool indexer_forward, bool indexer_reverse)
		{
			indexer_forward_ = indexer_forward;
			indexer_reverse_ = indexer_reverse;
		}
		bool indexer_forward_;
		bool indexer_reverse_;
};

class IndexerController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{
	public:
		IndexerController()
		{
		}

		//the four essential functions for a controller: init, starting, update, stopping

		virtual bool init(hardware_interface::RobotHW *hw,
						  ros::NodeHandle             &root_nh,
						  ros::NodeHandle             &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration &period) override;
		virtual void stopping(const ros::Time &time) override;
		bool cmdService (controllers_2020_msgs::IndexerSrv::Request &req, controllers_2020_msgs::IndexerSrv::Response &/*response*/);
		//void talonStateCallback(const talon_state_msgs::TalonState &talon_state);

	private:
		talon_controllers::TalonVelocityCloseLoopControllerInterface indexer_joint_; //interface for the indexer turning motor
		realtime_tools::RealtimeBuffer<IndexerCommand> indexer_cmd_;
		ros::ServiceServer indexer_service_;
		ros::Subscriber talon_state_sub_;

		double indexer_speed_;
}; //class

} //namespace

