#include "cargo_outtake_controller/cargo_outtake_controller.h"

namespace cargo_outtake_controller
{

	bool CargoOuttakeController::init(hardware_interface::PositionJointInterface *hw,
			ros::NodeHandle                 &/*root_nh*/,
			ros::NodeHandle                 &controller_nh)
	{
		//initialize cargo outtake clamp joint (pneumatic piston that controls clamp)
		cargo_outtake_clamp_joint_ = hw->getHandle("clamp_joint");

		cargo_outtake_service_ = controller_nh.advertiseService("cargo_outtake_command", &CargoOuttakeController::cmdService, this);

		return true;
	}

	void CargoOuttakeController::starting(const ros::Time &/*time*/) {
		// kicker, clamp
		cargo_outtake_cmd_.writeFromNonRT(CargoOuttakeCommand());
	}

	void CargoOuttakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		//process input for the up/down part of the intake (pneumatic piston)
		const CargoOuttakeCommand cargo_outtake_cmd = *(cargo_outtake_cmd_.readFromRT());

		double clamp_cmd_double; //to store processed input
		if(cargo_outtake_cmd.clamp_cmd_ == true) {
			clamp_cmd_double = 1;
		}
		else
		{
			clamp_cmd_double = 0;
		}
		//ROS_WARN("cargo outtake clamp command: " << clamp_cmd_double);

		cargo_outtake_clamp_joint_.setCommand(clamp_cmd_double); // set the in/out command to the up/down part of the outtake
	}

	void CargoOuttakeController::stopping(const ros::Time &/*time*/) {
	}

	bool CargoOuttakeController::cmdService(cargo_outtake_controller::CargoOuttakeSrv::Request &req, cargo_outtake_controller::CargoOuttakeSrv::Response &/*res*/) {
		if(isRunning())
		{
			cargo_outtake_cmd_.writeFromNonRT(CargoOuttakeCommand(req.clamp_release));
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. CargoOuttakeController is not running.");
			return false;
		}
		return true;
	}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(cargo_outtake_controller::CargoOuttakeController, controller_interface::ControllerBase)

