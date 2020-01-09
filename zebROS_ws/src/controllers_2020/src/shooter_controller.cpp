#include "controllers_2020/shooter_controller.h"

namespace shooter_controller
{
    bool ShooterController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {


        return true;
    }

    void ShooterController::starting(const ros::Time &/*time*/) {

    }

    void ShooterController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

    }

    void ShooterController::stopping(const ros::Time &/*time*/) {
    }


}//namespace

PLUGINLIB_EXPORT_CLASS(shooter_controller::ShooterController, controller_interface::ControllerBase)

