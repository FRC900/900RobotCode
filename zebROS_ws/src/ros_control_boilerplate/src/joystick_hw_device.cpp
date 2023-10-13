#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/../../src/joystick_devices.cpp"
#include "ros_control_boilerplate/../../src/joystick_device.cpp"

template class JoystickDevices<JoystickDevice>;