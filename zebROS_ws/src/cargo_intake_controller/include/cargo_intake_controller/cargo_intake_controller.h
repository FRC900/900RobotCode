#ifndef CARGO_INTAKE_CONTROLLER
#define CARGO_INTAKE_CONTROLLER

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <cargo_intake_controller/CargoIntakeSrv.h>

namespace cargo_intake_controller
{
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface

class CargoIntakeCommand //define class to hold command data - so we only need 1 realtime buffer
{
	public:
	CargoIntakeCommand()
		: spin_cmd_(0.0)
		, intake_arm_cmd_(false)
	{
	}
	CargoIntakeCommand(double spin_cmd, bool intake_arm_cmd)
	{
		spin_cmd_ = spin_cmd;
		intake_arm_cmd_ = intake_arm_cmd;
	}
	double spin_cmd_;
	bool intake_arm_cmd_;
}; //class

//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class CargoIntakeController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface, hardware_interface::PositionJointInterface>
{
        public:
            CargoIntakeController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh);
            virtual void starting(const ros::Time &time);
            virtual void update(const ros::Time & time, const ros::Duration& period);
            virtual void stopping(const ros::Time &time);

            virtual bool cmdService(cargo_intake_controller::CargoIntakeSrv::Request &req,
					                cargo_intake_controller::CargoIntakeSrv::Response &res);

        private:
            talon_controllers::TalonPercentOutputControllerInterface cargo_intake_joint_; //interface for the spinny part of the intake
			hardware_interface::JointHandle cargo_intake_arm_joint_; //interface for the up/down arm of the intake

			realtime_tools::RealtimeBuffer<CargoIntakeCommand> cargo_intake_cmd_;

            ros::ServiceServer cargo_intake_service_; //service for receiving commands
}; //class

} //namespace
#endif
