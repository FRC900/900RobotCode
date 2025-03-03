#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "periodic_interval_counter/periodic_interval_counter.h"

#include "hardware_interface/joint_command_interface.h"

namespace camera_trigger_controller_2025
{

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
class CameraTriggerController_2025 : public controller_interface::Controller<hardware_interface::JointCommandInterface> {
public:
    bool init(hardware_interface::JointCommandInterface *joint_command_interface,
              ros::NodeHandle & /*root_nh*/,
              ros::NodeHandle &controller_nh) override
    {
        ROS_INFO_STREAM("2025_camera_trigger_controller: init");

		//get publish rate from config file
        double publish_rate{60.};
		if (!controller_nh.param("publish_rate", publish_rate, publish_rate))
		{
			ROS_WARN("Could not read publish_rate in 2025_camera_trigger_controller");
		}
		else if (publish_rate <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in 2025_camera_trigger_controller (" << publish_rate << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate);

        joint_ = joint_command_interface->getHandle("camera_trigger");
        ROS_INFO_STREAM("2025_camera_trigger_controller: init successful");
        return true;
    }

    void starting(const ros::Time &time) override
    {
        ROS_INFO_STREAM("2025_camera_trigger_controller: starting");
		interval_counter_->reset();
    }

    void update(const ros::Time &time, const ros::Duration &duration) override
    {
        // Always add the time increment to the accumulated time.
        // Keep track of whether the desired period has been reached.
        timer_expired_ |= interval_counter_->update(duration);

        // Spec says trigger pulse can't be < 2uSec.
        // Controller is running at 250Hz, or 4mSec per update.  
        // So it should be fine to keep the pulse high for 1 update.
        if (joint_.getCommand() > 0.0)
        {
            joint_.setCommand(0.0);
        }
        // If the timer has expired, set the command to 1.0 to
        // trigger a high pulse for the camera trigger.
		else if (timer_expired_)
		{
            joint_.setCommand(1.0);
            timer_expired_ = false;
        }
    }

    void stopping(const ros::Time & /*time*/) override
    {
    }

private:
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
    hardware_interface::JointHandle joint_;

    bool timer_expired_{false};
}; // class

}//namespace

PLUGINLIB_EXPORT_CLASS(camera_trigger_controller_2025::CameraTriggerController_2025, controller_interface::ControllerBase)