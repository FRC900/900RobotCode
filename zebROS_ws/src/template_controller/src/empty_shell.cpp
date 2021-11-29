//TEMPLATE FOR WRITING A CONTROLLER
//replace "mech" with the name of your mechanism, words_separated_by_underscores
//replace "Mech" with the name of your mechanism, ThisIsTheFormatForThat

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>


namespace empty_controller
{
class EmptyController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{
public:
	bool init(hardware_interface::RobotHW *hw,
			  ros::NodeHandle             &/*root_nh*/,
			  ros::NodeHandle             &controller_nh) override
	{


		return true;
	}

	void starting(const ros::Time &/*time*/) override {

	}

	void update(const ros::Time &/*time*/, const ros::Duration &/*period*/) override {

	}

	void stopping(const ros::Time &/*time*/) override {
	}

};

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
#include <pluginlib/class_list_macros.h> //to compile as a controller
PLUGINLIB_EXPORT_CLASS(empty_controller::EmptyController, controller_interface::ControllerBase)
