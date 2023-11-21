#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/talonfxpro_devices.h"
#include "../src/talonfxpro_devices.cpp"
#include "../src/talonfxpro_device.cpp"

template class TalonFXProDevices<true>;