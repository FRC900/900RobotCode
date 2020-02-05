#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include "controllers_2020_msgs/ShooterSrv.h"
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Bool.h>

namespace shooter_controller
{


class ShooterCommand
{
	public:
		ShooterCommand()
			: set_velocity_(0.0),
			  shooter_hood_raise_(false)
	    {
		}
		ShooterCommand(double set_velocity, bool shooter_hood_raise)
		{
			set_velocity_ = set_velocity;
			shooter_hood_raise_ = shooter_hood_raise;
		}
		double set_velocity_;
		bool shooter_hood_raise_;
};
//this is the controller class, used to make a controller
class ShooterController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{
        public:
            ShooterController()
            {
            }

			//the four essential functions for a controller: init, starting, update, stopping

			virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

			bool cmdService(controllers_2020_msgs::ShooterSrv::Request &req,
							controllers_2020_msgs::ShooterSrv::Response &res);
        private:
			talon_controllers::TalonVelocityCloseLoopControllerInterface shooter_joint_;
			hardware_interface::JointHandle shooter_hood_joint_;

			ros::ServiceServer shooter_service_;
		    realtime_tools::RealtimeBuffer<ShooterCommand> cmd_buffer_;

                    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> ready_to_shoot_pub_; 
                    double time_to_raise_hood_;
                    double speed_threshhold_;
                    ros::Time last_command_time_;


}; //class

} //namespace

