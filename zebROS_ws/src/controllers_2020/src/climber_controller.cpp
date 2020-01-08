#include "climber_controller/climber_controller.h"

namespace climber_controller
{
    bool ClimberController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {


        return true;
    }

    void ClimberController::starting(const ros::Time &/*time*/) {

    }

    void ClimberController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

    }

    void ClimberController::stopping(const ros::Time &/*time*/) {
    }


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(climber_controller::ClimberController, controller_interface::ControllerBase)

