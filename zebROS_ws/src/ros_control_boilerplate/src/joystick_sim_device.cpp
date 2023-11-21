#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "../src/joystick_devices.cpp"
#include "../src/joystick_device.cpp"
#include "../src/sim_joystick_device.cpp"

template class JoystickDevices<true>;