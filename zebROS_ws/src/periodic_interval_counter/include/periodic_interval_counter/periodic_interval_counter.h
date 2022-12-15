// Code to track perioidic updates across multiple
// iterations. Mainly used to control the timing of rate-limited
// state controller publishers and other related hardware interface code
#ifndef PERIODIC_INTERVAL_COUNTER_INC__
#define PERIODIC_INTERVAL_COUNTER_INC__

#include <ros/console.h>
#include <ros/duration.h>

class PeriodicIntervalCounter
{
public:
    // Create a counter which updates at the requested frequency
    PeriodicIntervalCounter(const double frequency)
        : desired_period_{ros::Duration{1. / frequency}}
    {
        // Crash early if the frequency passed in is invalid
        if (desired_period_ <= ros::Duration{0})
        {
            ROS_ERROR("Periodic Interval Counter update called with desired period <= 0");
            throw std::invalid_argument("Periodic Interval Counter update called with desired period <= 0");
        }
    }

    // Check to see if the desired interval has been reached this
    // iteration.
    // Typical operation is to add the time increment to the accumulated
    // time. If that value exceeds the configured period, decrease the
    // accumulated time by one period and return true to indicate to
    // the caller the desired period has been reached.  Decrementing the
    // accumulated time will give a running total of how close we are to
    // the ideal time between intervals.
    // If the increment is negative, warn and return true. This should never 
    // happen, but catch it here if it does. TODO - error out?
    // If none of these are true, return false to let the caller know
    // this update() call's increment didn't cause the accumulated value
    // to exceed the desired period.
    bool update(const ros::Duration &increment)
    {
        if (increment < ros::Duration{0})
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "Periodic Interval Counter called with increment <= 0 (" << increment << ")");
            reset();
            return true;
        }
        current_interval_ += increment;
        if (current_interval_ > desired_period_)
        {
            current_interval_ -= desired_period_;
            return true;
        }
        return false;
    }

    // Clear out the accumulated time towards the desired interval,
    // starting the process from 0
    void reset(void)
    {
        current_interval_ = ros::Duration{0};
    }

    // Update the current accumulated time to equal the desired
    // update period, forcing a publish on the next update call
    void force_publish(void)
    {
        current_interval_ = desired_period_;
    }
private:
    // How often update() should return true, in seconds
    ros::Duration desired_period_;

    // Accumulated time towards reaching that desired period,
    // also in seconds
    ros::Duration current_interval_{0};
};

#endif