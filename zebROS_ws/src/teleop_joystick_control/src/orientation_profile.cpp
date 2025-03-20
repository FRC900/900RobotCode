#include "angles/angles.h"
#include "ros/console.h"
#include "teleop_joystick_control/orientation_profile.h"

OrientationProfile::OrientationProfile()
{
    // TODO : params from config for max velocity, acceleration and jerk
    ruckig_input_.max_velocity[0] = 6;
    ruckig_input_.max_acceleration[0] = 18;
    ruckig_input_.max_jerk[0] = 54;
}

bool OrientationProfile::createProfile(const OrientationState &current_state, const double target_orientation)
{
    // Prevent constantly regenrating the same trajectory
    if (most_recent_was_trajectory_ && (target_orientation == target_state_.position))
    {
        return true;
    }
    ROS_WARN_STREAM("Generating profile for angle " << target_orientation);
    initial_state_ = OrientationState(angles::normalize_angle(current_state.position), current_state.velocity, current_state.acceleration);
    target_state_ = OrientationState(angles::normalize_angle(target_orientation), 0, 0);
    start_time_ = ros::Time::now();

    // Profile is delta angle from initial state position
    ruckig_input_.current_position[0] = 0;
    ruckig_input_.current_velocity[0] = current_state.velocity;
    ruckig_input_.current_acceleration[0] = current_state.acceleration;
    ruckig_input_.target_position[0] = angles::shortest_angular_distance(current_state.position, target_orientation);
    ruckig_input_.target_velocity[0] = 0;
    ruckig_input_.target_acceleration[0] = 0;
    const auto result = ruckig_obj_.calculate(ruckig_input_, ruckig_trajectory_);
    if (result != ruckig::Result::Working)
    {
        ROS_ERROR_STREAM("Failed to create trajectory: " << static_cast<int>(result));
        return false;
    }
    trajectory_running_ = true;
    most_recent_was_trajectory_ = true;
    return true;
}

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