#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

#include "hardware_interface/joint_command_interface.h"
#include "controller_interface/multi_interface_controller.h"
#include "talon_controllers/talon_controller_interface.h"

#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
//#include <controllers_2019_msgs/CargoIntakeSrv.h>

namespace indexer_test_controller
{
enum IndexerStates
{
	IndexerState_NotShooting_Idle,
	IndexerState_NotShooting_RunMotor1,
	IndexerState_NotShooting_RunMotor2,
	IndexerState_NotShooting_DelayBeforeStopMotor
};


//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface

//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class IndexerTestController :
	public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface,hardware_interface::JointStateInterface>
{
	public:
		IndexerTestController(void)
		{
		}

		virtual bool init(hardware_interface::RobotHW *hw,
						  ros::NodeHandle             &root_nh,
						  ros::NodeHandle             &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time & time, const ros::Duration& period) override;
		virtual void stopping(const ros::Time &time) override;

		void shooterReadyCallback(const std_msgs::Bool &msg);

	private:
		talon_controllers::TalonPercentOutputControllerInterface indexer_motor_joint_;       //interface for the spinny part of the indexer
		hardware_interface::JointStateHandle                     indexer_linebreak_1_joint_; // TBD
		hardware_interface::JointStateHandle                     indexer_linebreak_2_joint_; // TBD
		hardware_interface::JointStateHandle                     indexer_linebreak_3_joint_; // TBD
		hardware_interface::JointStateHandle                     indexer_linebreak_4_joint_; // TBD

		realtime_tools::RealtimeBuffer<bool>      shooter_ready_cmd_;
		ros::Subscriber                           shooter_ready_callback_;

		IndexerStates                             indexer_state_;
		ros::Time                                 delay_before_stop_motor_start_time_;

		ddynamic_reconfigure::DDynamicReconfigure ddr_;
		double                                    indexer_motor_speed_;
		double                                    delay_before_stop_motor_;

}; //class

} //namespace
