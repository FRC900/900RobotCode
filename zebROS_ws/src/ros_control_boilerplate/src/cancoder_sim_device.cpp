#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "../src/cancoder_devices.cpp"
#include "../src/cancoder_device.cpp"

template class CANCoderDevices<true>;
template class CANCoderDevice<true>;
