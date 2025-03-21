#include "angles/angles.h"
#include "ros/console.h"
#include "teleop_joystick_control/orientation_profile.h"

OrientationProfile::OrientationProfile(const ros::NodeHandle &nh)
    : ddr_{ros::NodeHandle(nh, "orientation_profile")}
{
    // Feels dumb to repeat this twice (once for DDR, once here), but
    // the other choice is to make a NodeHandle class member var which
    // is only used in the constructor.
	ros::NodeHandle n_params(nh, "orientation_profile");
    n_params.param<double>("velocity", ruckig_input_.max_velocity[0], 6.0);
    n_params.param<double>("acceleration", ruckig_input_.max_acceleration[0], 12.0);
    n_params.param<double>("jerk", ruckig_input_.max_jerk[0], 64.0);

    ddr_.registerVariable<double>("velocity", &ruckig_input_.max_velocity[0], "Orientation profile velocity", 0.0, 20.0);
    ddr_.registerVariable<double>("acceleration", &ruckig_input_.max_acceleration[0], "Orientation profile acceleration", 0.0, 200.0);
    ddr_.registerVariable<double>("jerk", &ruckig_input_.max_jerk[0], "Orientation profile jerk", 0.0, 1000.0);

    ddr_.publishServicesTopics();

    // These are the same for all trajectories, so set them once here and leave them alone
    ruckig_input_.current_position[0] = 0;
    ruckig_input_.current_velocity[0] = 0; 
    ruckig_input_.current_acceleration[0] = 0; 

    ruckig_input_.target_velocity[0] = 0;
    ruckig_input_.target_acceleration[0] = 0;
}

bool OrientationProfile::createProfile(const double current_orientation, const double target_orientation)
{
    // Prevent constantly regenrating the same trajectory
    const auto normalized_taget_orientation = angles::normalize_angle(target_orientation);
    if (most_recent_was_trajectory_ && (normalized_taget_orientation == target_state_.position))
    {
        return true;
    }
    // TODO : if we can get a reliable angular velocity from the robot (e.g. from swerve odom
    // or maybe the commanded angular velocity from the PID node), we can use that as the
    // initial velocity for the profile. We'd have to increase the accel and jerk limits to
    // prevent the robot from overshooting in one direction before snapping back the other way.
    initial_state_ = OrientationState(angles::normalize_angle(current_orientation), 0, 0);
    target_state_ = OrientationState(normalized_taget_orientation, 0, 0);

    // Profile is delta angle from initial state position. The profile goes from 0
    // to delta angle in the nearest driection.  The code which returns the angle
    // to drive to is responsible for adding the initial state position back in.
    ruckig_input_.target_position[0] = angles::shortest_angular_distance(current_orientation, target_orientation);
    const auto result = ruckig_obj_.calculate(ruckig_input_, ruckig_trajectory_);
    if (result != ruckig::Result::Working)
    {
        ROS_ERROR_STREAM("Failed to create trajectory: " << static_cast<int>(result));
        trajectory_running_ = false;
        return false;
    }
    start_time_ = ros::Time::now();
    trajectory_running_ = true;
    most_recent_was_trajectory_ = true;
    return true;
}

// Set a non-profiled target state.
// That is, the robot will just seek the target state using pure PID (plus
// a static FF term if state.velocity is non-zero). This is used in cases
// where we are holding a position set via teleop control or when following
// e.g. a path follower path where the profile is generated elsewhere.
void OrientationProfile::setOrientationTarget(const OrientationState &state)
{
    target_state_ = OrientationState(angles::normalize_angle(state.position), state.velocity, state.acceleration);
    trajectory_running_ = false;
    most_recent_was_trajectory_ = false;
}

OrientationState OrientationProfile::getOrientationState(void)
{
    const auto now = ros::Time::now(); // Check to see if we're finished with the trajectory
    if (trajectory_running_)
    {
        const auto trajectory_time = (now - start_time_).toSec();
        if (trajectory_time >= ruckig_trajectory_.get_duration())
        {
            trajectory_running_ = false;
        }
    }
    if (!trajectory_running_) // Non-profiled motion just seeks the target state using pure PID 
    {
        return target_state_;
    }
    // Sample the trajectory at the current time
    std::array<double, 1> new_position;
    std::array<double, 1> new_velocity;
    std::array<double, 1> new_acceleration;
    ruckig_trajectory_.at_time((now - start_time_).toSec(), new_position, new_velocity, new_acceleration);
    // Profile is delta angle from initial state position, so add that back
    // in here to give an absolute angle to target
    return OrientationState(angles::normalize_angle(new_position[0] + initial_state_.position), new_velocity[0], new_acceleration[0]);
}