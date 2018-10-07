#include "arm_controller/arm_controller.h"
namespace arm_controller
{
	bool ArmController::init(hardware_interface::RobotHW *hw,
						ros::NodeHandle		&controller_nh)
	{
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
	return true;

}
	void ArmController::starting(const ros::Time &time){
	ROS_WARN("arm controller working");
	}
	void ArmController::update(const ros::Time &time, const ros::Duration& period){
		double command = *command_.readFromRT();	
		arm_joint.setCommand(command);
	}
}//Namespace

