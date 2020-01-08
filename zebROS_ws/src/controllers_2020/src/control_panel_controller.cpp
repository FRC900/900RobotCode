#include "control_panel_controller/control_panel_controller.h"

namespace control_panel_controller
{
    bool Control_Panel_Controller::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {


        return true;
    }

    void Control_Panel_Controller::starting(const ros::Time &/*time*/) {

    }

    void Control_Panel_Controller::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

    }

    void Control_Panel_Controller::stopping(const ros::Time &/*time*/) {
    }


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(control_panel_controller::Control_Panel_Controller, controller_interface::ControllerBase)

