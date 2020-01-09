#include "controllers_2020/intake_controller.h"

namespace intake_controller
{
    bool IntakeController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {


        return true;
    }

    void IntakeController::starting(const ros::Time &/*time*/) {

    }

    void IntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

    }

    void IntakeController::stopping(const ros::Time &/*time*/) {
    }


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(intake_controller::IntakeController, controller_interface::ControllerBase)

