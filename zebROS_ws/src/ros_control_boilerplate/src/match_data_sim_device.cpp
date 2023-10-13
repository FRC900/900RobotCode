#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/../../src/match_data_devices.cpp"
#include "ros_control_boilerplate/../../src/match_data_device.cpp"
#include "ros_control_boilerplate/../../src/sim_match_data_device.cpp"

template class MatchDataDevices<SimMatchDataDevice>;