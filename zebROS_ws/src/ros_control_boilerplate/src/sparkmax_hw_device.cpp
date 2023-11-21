#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "../src/sparkmax_devices.cpp"
#include "../src/sparkmax_device.cpp"

template class SparkMaxDevices<false>;
template class SparkMaxDevice<false>;