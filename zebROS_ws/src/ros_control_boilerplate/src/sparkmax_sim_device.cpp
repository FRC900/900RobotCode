#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/../../src/sparkmax_devices.cpp"
#include "ros_control_boilerplate/../../src/sparkmax_device.cpp"

template class SparkMaxDevices<true>;
template class SparkMaxDevice<true>;