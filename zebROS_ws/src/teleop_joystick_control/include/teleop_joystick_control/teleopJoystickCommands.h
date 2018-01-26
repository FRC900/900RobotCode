#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "teleop_joystick_control/RobotState.h"
#include "talon_controllers/CloseLoopControllerMsg.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "teleop_joystick_control/teleopJoystickCommands.h"
#include "ros/time.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <string>


void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &msg);
void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble);
void evaluateTime(const ros_control_boilerplate::MatchSpecificData::ConstPtr &msg);



