#include "controllers_2020/control_panel_controller.h"

namespace control_panel_controller
{
    bool ControlPanelController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {


        return true;
    }

    void ControlPanelController::starting(const ros::Time &/*time*/) {

    }

    void ControlPanelController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

    }

    void ControlPanelController::stopping(const ros::Time &/*time*/) {
    }


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(control_panel_controller::ControlPanelController, controller_interface::ControllerBase)

