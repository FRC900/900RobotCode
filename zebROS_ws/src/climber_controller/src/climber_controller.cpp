#include "climber_controller/climber_controller.h"

namespace climber_controller
{

bool ClimberController::init(hardware_interface::PositionJointInterface *hw,
							 ros::NodeHandle                            &/*root_nh*/,
							 ros::NodeHandle                            &controller_nh)
{
    feet_retract_ = hw->getHandle("climber_feet_retract");
    ski_retract_ = hw->getHandle("climber_ski_joint");

	// Hack - abuse boost bind to pass a 3rd argument to the callback
	// indicating which realtime buffer to write to. Since that's the
	// only thing which changes between the callbacks, it should save
	// a bit of code
	feet_retract_service_ = controller_nh.advertiseService<
		std_srvs::SetBool::Request,
		std_srvs::SetBool::Response	>
			("climber_feet_retract", boost::bind(&ClimberController::activateSrv, this, _1, _2, boost::ref(feet_retract_cmd_)));

    release_endgame_ = hw->getHandle("climber_release_endgame");
	release_endgame_service_ = controller_nh.advertiseService<
		std_srvs::SetBool::Request,
		std_srvs::SetBool::Response	>
			("climber_release_endgame", boost::bind(&ClimberController::activateSrv, this, _1, _2, boost::ref(release_endgame_cmd_)));
    return true;
}

void ClimberController::starting(const ros::Time &/*time*/) {
	feet_retract_cmd_.writeFromNonRT(false);
	release_endgame_cmd_.writeFromNonRT(false);
}

void ClimberController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
    const bool feet_retract_cmd = *(feet_retract_cmd_.readFromRT());
    const bool release_endgame_cmd = *(release_endgame_cmd_.readFromRT());

    double feet_retract_cmd_double;
	double ski_retract_cmd_double;
	if(feet_retract_cmd == true)
	{
		feet_retract_cmd_double = 1;
		ski_retract_cmd_double = 1;
	}
	else if(feet_retract_cmd == false)
	{
		feet_retract_cmd_double = -1;
		ski_retract_cmd_double = 0;
	}

    feet_retract_.setCommand(feet_retract_cmd_double); //retract the feet
    ski_retract_.setCommand(ski_retract_cmd_double); //retract the piston to make the ski deploy

    double release_endgame_cmd_double;
	if(release_endgame_cmd == true)
	{
		release_endgame_cmd_double = 1;
	}
	else if(release_endgame_cmd == false)
	{
		release_endgame_cmd_double = -1;
	}

    release_endgame_.setCommand(release_endgame_cmd_double); //release the endgame
}

void ClimberController::stopping(const ros::Time &/*time*/) {
}

// Callback takes the request, writes it into the
// realtime buffer
bool ClimberController::activateSrv(std_srvs::SetBool::Request &req,
									std_srvs::SetBool::Response &/*response*/,
									realtime_tools::RealtimeBuffer<bool> &realtime_buffer)
{
    if(isRunning())
    {
        realtime_buffer.writeFromNonRT(req.data);
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. ClimberController is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(climber_controller::ClimberController, controller_interface::ControllerBase)
