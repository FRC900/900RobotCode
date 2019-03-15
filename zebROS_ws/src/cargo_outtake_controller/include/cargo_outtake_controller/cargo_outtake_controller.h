#ifndef CARGO_OUTTAKE_CONTROLLER
#define CARGO_OUTTAKE_CONTROLLER

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
#include <cargo_outtake_controller/CargoOuttakeSrv.h>

namespace cargo_outtake_controller
{

	// TODO : redo this to match the new cargo hardware, when available
class CargoOuttakeCommand //define class to hold command data - so we only need 1 realtime buffer
{
	public:
		// Use this to set the default state of the mechanism
		CargoOuttakeCommand()
			: clamp_cmd_(false)
		{
		}
		CargoOuttakeCommand(bool clamp_cmd)
		{
			clamp_cmd_ = clamp_cmd;
		}
		bool clamp_cmd_;
}; //class

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class CargoOuttakeController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            CargoOuttakeController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh);
            virtual void starting(const ros::Time &time);
            virtual void update(const ros::Time & time, const ros::Duration& period);
            virtual void stopping(const ros::Time &time);

            virtual bool cmdService(cargo_outtake_controller::CargoOuttakeSrv::Request &req,
                                    cargo_outtake_controller::CargoOuttakeSrv::Response &res);

        private:
            std::vector<std::string> joint_names_; //still not used, but we might have to for config file things?
            //hardware_interface::JointHandle cargo_outtake_kicker_joint_; //interface for the kicker of the outtake
			hardware_interface::JointHandle cargo_outtake_clamp_joint_; //interface for the clamp of the outtake

			realtime_tools::RealtimeBuffer<CargoOuttakeCommand> cargo_outtake_cmd_;

            ros::ServiceServer cargo_outtake_service_; //service for receiving commands
}; //class
} //namespace
#endif
