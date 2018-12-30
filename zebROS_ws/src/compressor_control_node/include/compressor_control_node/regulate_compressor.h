#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include "ros/time.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <string>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <match_state_controller/MatchSpecificData.h>
#include <pdp_state_controller/PDPData.h>

//BE WARNED. THIS NODE USES IMPERIAL UNITS..........

void pressureCallback(const sensor_msgs::JointState &joint_state);
void matchDataCallback(const match_state_controller::MatchSpecificData &matchData);
void currentCallback(const pdp_state_controller::PDPData &current);

