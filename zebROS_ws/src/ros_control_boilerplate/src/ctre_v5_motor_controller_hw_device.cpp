#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "../src/ctre_v5_motor_controllers.cpp"
#include "../src/ctre_v5_motor_controller.cpp"

template class CTREV5MotorControllers<false>;