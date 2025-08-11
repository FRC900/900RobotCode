#ifndef ORIENTATION_PROFILE_INC__
#define ORIENTATION_PROFILE_INC__

#include <ros/node_handle.h>
#include <ros/time.h>
#include <ruckig/ruckig.hpp>
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

// Store orientation state
// Currently only position and velocity are used, but accel is
// generated and tracked when using profile mode
struct OrientationState
{
    OrientationState() = default;
    OrientationState(double position, double velocity, double acceleration = 0.0)
        : position(position), velocity(velocity), acceleration(acceleration)
    {
    }
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

// Contains data and state for the orientation state,
// including the current state and the target state
// along with info needed to get current orientation
// and velocity for profiled motion when snapping to
// and angle.
class OrientationProfile
{
public:
    explicit OrientationProfile(const ros::NodeHandle &nh);

    // This sets at target state with profiling enabled, including generating a trajectory to follow
    bool createProfile(const double current_orientation, const double target_orientation);

    // This sets a target state with profiling disabled
    void setOrientationTarget(const OrientationState &state);

    // Gets the current state of the orientation profile right now.
    // For non-profiled motion this just returns the target state
    // set in setOrientationTarget.
    // For profiled motion, it returns the current desired 
    // state (position and velocity) to track to stay on 
    // the generated orientation trajectory.
    OrientationState getOrientationState(void);

private:
    // For these, the pattern is to normalize angles to [-pi, pi) when
    // setting the values, as well as when returning trajectory waypoints
    // from getOrientationState. This is to ensure that the trajectory
    // is generated correctly and that the robot doesn't try to rotate
    // more than 180 degrees to reach a target angle.
    OrientationState initial_state_;
    OrientationState target_state_;
    ros::Time start_time_;

    // Is a trajectory currently running?
    // False if a target state has been set without profiling
    // and also false if a profile was running but has ended
    bool trajectory_running_{false};
    
    // Was the most recent target set with a trajectory (e.g. via createProfile)?
    // This is used to prevent re-generating a trajectory
    // if the target is set to the same angle as the last
    // target.
    bool most_recent_was_trajectory_{false};

    // Various state for the Ruckig motion profile generator
    ruckig::Ruckig<1> ruckig_obj_;
    ruckig::InputParameter<1> ruckig_input_;
    ruckig::Trajectory<1> ruckig_trajectory_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
};

#endif
