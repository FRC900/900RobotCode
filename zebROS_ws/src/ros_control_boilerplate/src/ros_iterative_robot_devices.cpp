#include "periodic_interval_counter/periodic_interval_counter.h"
#include "ros_control_boilerplate/ros_iterative_robot_devices.h"
#include "ros_control_boilerplate/ros_iterative_robot.h"

ROSIterativeRobotDevices::ROSIterativeRobotDevices(ros::NodeHandle &root_nh)
    : ros_iterative_robot_{std::make_unique<ros_control_boilerplate::ROSIterativeRobot>()}
{
    double read_hz{50};
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("robot_iteration_hz", read_hz, read_hz)) 
    {
		ROS_WARN("Failed to read robot_iteration_hz in frc_robot_interface");
	}
    interval_counter_ = std::make_unique<PeriodicIntervalCounter>(read_hz);
    ROS_INFO_STREAM("Creating ROSIterativeRobot with a " << read_hz << "Hz update rate");
}

ROSIterativeRobotDevices::~ROSIterativeRobotDevices() = default;

hardware_interface::InterfaceManager *ROSIterativeRobotDevices::registerInterface()
{
    return &interface_manager_;
}

void ROSIterativeRobotDevices::read(const ros::Time& /*time*/, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("OneIteration");
    if (isReady())
    {
        if (!started_competition_)
        {
            ros_iterative_robot_->StartCompetition();
        }
        if (interval_counter_->update(period))
        {
            ros_iterative_robot_->OneIteration();
        }
    }
}