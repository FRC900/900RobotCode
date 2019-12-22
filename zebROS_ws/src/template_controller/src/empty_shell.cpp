//TEMPLATE FOR WRITING A CONTROLLER
//replace "mech" with the name of your mechanism, words_separated_by_underscores
//replace "Mech" with the name of your mechanism, ThisIsTheFormatForThat


#include "mech_controller/mech_controller.h"

namespace mech_controller
{
	bool MechController::init(hardware_interface::RobotHW *hw,
									 ros::NodeHandle                 &/*root_nh*/,
									 ros::NodeHandle                 &controller_nh)
	{


		return true;
	}

	void MechController::starting(const ros::Time &/*time*/) {

	}

	void MechController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

	}

	void MechController::stopping(const ros::Time &/*time*/) {
	}


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(mech_controller::MechController, controller_interface::ControllerBase)
