#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>


#include <controllers_2020_msgs/IntakeArmSrv.h>
#include <controllers_2020_msgs/IntakeRollerSrv.h>
#include <std_srvs/SetBool.h>

namespace intake_controller
{



//this is the controller class, used to make a controller
class IntakeController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{
        public:
            IntakeController()
            {
            }

			//the four essential functions for a controller: init, starting, update, stopping

			virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;
        private:
            bool cmdServiceArm (controllers_2020_msgs::IntakeArmSrv::Request &req, controllers_2020_msgs::IntakeArmSrv::Response &);
            bool cmdServiceRoller (controllers_2020_msgs::IntakeRollerSrv::Request &req, controllers_2020_msgs::IntakeRollerSrv::Response &);

			bool disableIntakeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &);

			talon_controllers::TalonPercentOutputControllerInterface intake_joint_;//intake for intake motor
			hardware_interface::JointHandle intake_arm_joint_;//interface for intake arm solenoid
			realtime_tools::RealtimeBuffer<bool> arm_extend_cmd_buffer_;
			realtime_tools::RealtimeBuffer<double> percent_out_cmd_buffer_;
			ros::ServiceServer intake_arm_service_;
			ros::ServiceServer intake_roller_service_;

			ros::ServiceServer intake_disable_service_;
			realtime_tools::RealtimeBuffer<bool> forward_disabled_; //set to true by the indexer server when it's finishing up properly storing a ball, to ensure the proper gap



}; //class

} //namespace

