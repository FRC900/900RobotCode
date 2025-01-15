#include <pluginlib/class_list_macros.h>
#include <simulator_interface/simulator_base.h>
#include <general_simulators/default_simulator.h>
#include <general_simulators/flywheel_simulator.h>
#include <general_simulators/elevator_simulator.h>
#include <general_simulators/single_jointed_arm_simulator.h>
#include <general_simulators/swerve_simulator.h>

PLUGINLIB_EXPORT_CLASS(general_simulators::DefaultSimulator, simulator_base::Simulator)
PLUGINLIB_EXPORT_CLASS(general_simulators::FlywheelSimulator, simulator_base::Simulator)
PLUGINLIB_EXPORT_CLASS(general_simulators::ElevatorSimulator, simulator_base::Simulator)
// PLUGINLIB_EXPORT_CLASS(general_simulators::SingleJointedArmSimulator, simulator_base::Simulator)
// PLUGINLIB_EXPORT_CLASS(general_simulators::SwerveSimulator<4>, simulator_base::Simulator)
// PLUGINLIB_EXPORT_CLASS(general_simulators::SwerveSimulator<1>, simulator_base::Simulator)