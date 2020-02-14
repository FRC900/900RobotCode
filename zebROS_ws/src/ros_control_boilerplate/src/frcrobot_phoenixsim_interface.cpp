#include <ros_control_boilerplate/frcrobot_phoenixsim_interface.h>

/* additional include for sim */
#include <ctre/phoenix/cci/Platform_CCI.h> // c_SimCreate
#include <ctre/phoenix/cci/Unmanaged_CCI.h> // c_FeedEnable

#include <HALInitializer.h>

namespace frcrobot_control
{
FRCRobotPhoenixSimInterface::FRCRobotPhoenixSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: FRCRobotHWInterface(nh, urdf_model)
{
}

bool FRCRobotPhoenixSimInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	for (size_t i = 0; i < can_ctre_mc_can_ids_.size(); i++)
	{
		// TODO : fix device type
		c_SimCreate(DeviceType::TalonSRXType, can_ctre_mc_can_ids_[i]);
		ROS_INFO_STREAM("phoenixsim : creating DeviceType::TalonSRXType id=" << can_ctre_mc_can_ids_[i]);
	}

	// for now we need a delay so backend can properly setup device properties
	// before we attempt creating API objects
	// this may not be necessary anymore
	ros::Duration(1.0).sleep();

	hal::init::InitializeHAL();
	if (!FRCRobotHWInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : FRCRobotHWInterface::init() returned false");
		return false;
	}
	return true;
}


// Write should grab the motor output and use it to run 1 timestep
// of the underlying motor sim
void FRCRobotPhoenixSimInterface::write(const ros::Time& time, const ros::Duration& period)
{
	c_FeedEnable(500);
	FRCRobotHWInterface::write(time, period);

	// Update the motor connected to each Talon here?
}
} // namespace
