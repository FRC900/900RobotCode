#include <pluginlib/class_list_macros.h>
#include <simulator_interface/simulator_base.h>
#include <simulator_interface/flywheel_simulator.h>

// TODO refactor sims to a different package so a rebuild doesn't rebuild ros_control_boilerplate

PLUGINLIB_EXPORT_CLASS(general_simulators::FlywheelSimulator, simulator_base::Simulator)