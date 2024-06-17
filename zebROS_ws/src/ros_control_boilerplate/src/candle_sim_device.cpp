#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "../src/candle_devices.cpp"
#include "../src/candle_device.cpp"

template class CANdleDevices<true>;
template class CANdleDevice<true>;
