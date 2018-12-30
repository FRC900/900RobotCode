#include "joint_state_listener/state_listener_controller.h"

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(state_listener_controller::JointStateListenerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(state_listener_controller::PDPStateListenerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(state_listener_controller::MatchStateListenerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(state_listener_controller::IMUStateListenerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(state_listener_controller::TalonStateListenerController,
					   controller_interface::ControllerBase)
