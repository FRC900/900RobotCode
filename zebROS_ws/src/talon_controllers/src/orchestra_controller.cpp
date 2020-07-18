#include "talon_controllers/orchestra_controller.h"

namespace orchestra_controller
{

bool OrchestraController::init(hardware_interface::OrchestraCommandInterface *hw,
								ros::NodeHandle								&root_nh,
								ros::NodeHandle								&controller_nh)
{
	ROS_INFO_STREAM("Orchestra controller init");

	std::vector<std::string> orchestra_names = hw->getNames();
	if (orchestra_names.size() > 1) {
		ROS_ERROR_STREAM("Cannot initialize multiple orchestras");
		return false;
        }
	else if (orchestra_names.size() < 1) {
		ROS_ERROR_STREAM("Cannot initialize zero orchestras");
		return false;
	}
	const std::string orchestra_name = orchestra_names[0];

	orchestra_command_handle_ = hw->getHandle(orchestra_name);  // throws on failure

	ROS_ERROR_STREAM("LOADING ALL OF THE SERVERS IN ORCHESTRA CONTROLLER");

	load_music_server_ = controller_nh.advertiseService("load_music", &OrchestraController::loadMusicService, this);
	set_state_server_ = controller_nh.advertiseService("set_state", &OrchestraController::setStateService, this);
	load_instruments_server_ = controller_nh.advertiseService("load_instruments", &OrchestraController::reloadInstrumentsService, this);
	return true;
}

void OrchestraController::starting(const ros::Time &time)
{
	state_.writeFromNonRT(2);
	instruments_.writeFromNonRT({});
	music_file_path_.writeFromNonRT("");

	previous_state_ = 2;
}

void OrchestraController::update(const ros::Time &time, const ros::Duration &)
{
	int state = *(state_.readFromRT());
	if(state != previous_state_)
	{
		ROS_INFO_STREAM("Changed state in OrchestraController");
		//TODO make enum
		switch(state)
		{
			case 0:
				ROS_INFO_STREAM("Playing music in OrchestraController");
				orchestra_command_handle_->play();
				break;
			case 1:
				ROS_INFO_STREAM("Pausing music in OrchestraController");
				orchestra_command_handle_->pause();
				break;
			case 2:
				ROS_INFO_STREAM("Stopping music in OrchestraController");
				orchestra_command_handle_->stop();
				break;
			default:
				ROS_ERROR_STREAM("Orchestra state must be 0, 1, or 2");
				break;
		}
		previous_state_ = state;
	}

	std::vector<std::string> instruments = *(instruments_.readFromRT());
	orchestra_command_handle_->addInstruments(instruments);

	std::string music_file_path = *(music_file_path_.readFromRT());
	orchestra_command_handle_->loadMusic(music_file_path);
}

void OrchestraController::stopping(const ros::Time & )
{}

bool OrchestraController::loadMusicService(talon_controller_msgs::LoadMusicSrv::Request &req,
		talon_controller_msgs::LoadMusicSrv::Response &res)
{
	if(isRunning())
	{
		music_file_path_.writeFromNonRT(req.music_path);
		res.success = true;
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. OrchestraController is not running.");
		return false;
	}
}


bool OrchestraController::setStateService(talon_controller_msgs::SetOrchestraStateSrv::Request &req,
		talon_controller_msgs::SetOrchestraStateSrv::Response &res)
{
	if(isRunning())
	{
		if(req.state == 0 || req.state == 1 || req.state == 2)
		{
			state_.writeFromNonRT(req.state);
			res.success = true;
			return true;
		}
		else
		{
			ROS_ERROR_STREAM("State must be 0, 1, or 2");
			res.success = false;
			return false;
		}
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. OrchestraController is not running.");
		return false;
	}
}

bool OrchestraController::reloadInstrumentsService(talon_controller_msgs::LoadInstrumentsSrv::Request &req,
		talon_controller_msgs::LoadInstrumentsSrv::Response &res)
{
	if(isRunning())
	{
		instruments_.writeFromNonRT(req.instruments);
		res.success = true;
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. OrchestraController is not running.");
		return false;
	}
}

}

PLUGINLIB_EXPORT_CLASS( orchestra_controller::OrchestraController, controller_interface::ControllerBase)
