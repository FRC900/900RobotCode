#ifndef ORIENTATION_PROFILE_INC__
#define ORIENTATION_PROFILE_INC__

#include <ros/time.h>
#include <ruckig/ruckig.hpp>

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

class OrientationProfile
{
public:
    explicit OrientationProfile();

    // This sets at target state with profiling enabled, including generating a trajectory to follow
    bool createProfile(const OrientationState &current_state, const double target_orientation);

    // This sets a target state with profiling disabled
    void setOrientationTarget(const OrientationState &state);
    OrientationState getOrientationState(void);

private:
    OrientationState initial_state_;
    OrientationState target_state_;
    ros::Time start_time_;
    bool trajectory_running_{false};
    bool most_recent_was_trajectory_{false};

    ruckig::Ruckig<1> ruckig_obj_;
    ruckig::InputParameter<1> ruckig_input_;
    ruckig::Trajectory<1> ruckig_trajectory_;

};

#endif
