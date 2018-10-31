#ifndef ARM_CONTROLLER
#define ARM_CONTROLLER

#include <ros/ros.h>
#include <vector>
#include <hardware_interface/joint_state_interface.h> //other than talon data
#include <realtime_tools/realtime_publisher.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h> //for writing controllers
#include <talon_interface/talon_state_interface.h> // "
#include <talon_controllers/talon_controller.h> // "
#include <talon_controllers/talon_controller_interface.h> // "
#include <arm_controller/SetArmState.h>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller

namespace arm_controller
{ 
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class ArmController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface>
{
	public:
		ArmController()
		{
		}

		//should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
		//answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
		virtual bool init(hardware_interface::RobotHW *hw,
							ros::NodeHandle						&root_nh,
							ros::NodeHandle						&controller_nh);
		virtual void starting(const ros::Time &time);
		virtual void update(const ros::Time & time, const ros::Duration& period);
		virtual void stopping(const ros::Time &time);
		
		//define function that executes the service
		virtual bool cmdService(arm_controller::SetArmState::Request &req, arm_controller::SetArmState::Response &res);
		

	private:
        	//std::vector<std::string> joint_names; //not used yet
		talon_controllers::TalonMotionMagicCloseLoopControllerInterface arm_joint_; //interface for the actual joint 
		ros::ServiceServer arm_state_service_;
                std::vector<double> arm_positions_;

                realtime_tools::RealtimeBuffer<int> service_command_; //stores most recent request value for the arm angle, in degrees
}; //class

} //namespace
#endif
