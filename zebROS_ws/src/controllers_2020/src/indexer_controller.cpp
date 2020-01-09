#include "controllers_2020/indexer_controller.h"

namespace indexer_controller
{
    bool IndexerController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {


        return true;
    }

    void IndexerController::starting(const ros::Time &/*time*/) {

    }

    void IndexerController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

    }

    void IndexerController::stopping(const ros::Time &/*time*/) {
    }


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(indexer_controller::IndexerController, controller_interface::ControllerBase)

