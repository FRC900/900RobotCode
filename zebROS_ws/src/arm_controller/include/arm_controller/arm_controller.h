#ifndef ARM_CONTROLLER
#define ARM_CONTROLLER

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <talon_controllers/CloseLoopControllerMsg.h>
namespace arm_controller
{

	class ArmController : public controller_interface::Controller<hardware_interface::TalonCommandInterface>{

	public:
		ArmController()
		{
		}

		virtual bool init(hardware_interface::RobotHW *hw,
							ros::NodeHandle			&controller_nh);
		virtual void starting(const ros::Time &time);
		virtual void update(const ros::Time & time, const ros::Duration& period);
		void stopping(const ros::Time& time) {}
	private:
		std::vector<std::string> joint_names;
		talon_controllers::TalonPositionCloseLoopControllerInterface arm_joint;
		realtime_tools::RealtimeBuffer<double> command_;
		
};
}
#endif
