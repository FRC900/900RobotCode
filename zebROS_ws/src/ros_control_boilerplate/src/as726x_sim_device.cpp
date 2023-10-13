#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/../../src/as726x_devices.cpp"
#include "ros_control_boilerplate/../../src/as726x_device.cpp"

template class AS726xDevices<true>;
template class AS726xDevice<true>;